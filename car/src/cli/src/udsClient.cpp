#include "udsClient.h"

#include <assert.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/epoll.h>
#include <chrono>

#include "ros/ros.h"
#include "roscar_common/error.h"
#include "roscar_common/rcmp.h"
#include "roscar_common/sessionBuffer.hpp"

using namespace std;
using namespace rapidjson;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace cli
{

const int UDSClient::INTERVAL_EPOLL_RETRY = 100;
const int UDSClient::INTERVAL_CONNECT_RETRY = 7;

UDSClient::UDSClient(OnSigCallbak onSigCallbak)
    : mUdsUri(), mStopFlag(true), mOnSigCallbak(onSigCallbak)
{
}

UDSClient::~UDSClient()
{
    stop();
}

bool UDSClient::start(const char *udsUri)
{
    mUdsUri = udsUri;

    // start threads
    {
        ROS_DEBUG("[UDSClient::start] start threads");
        lock_guard<mutex> lock(mAccessMutex);
        if (!mStopFlag)
        {
            ROS_ERROR("[UDSClient::start] stop thread first");
            return false;
        }

        mStopFlag = false;
        mThreadArray.emplace_back(&UDSClient::threadFunc, this);
    }

    return true;
}

void UDSClient::stop()
{
    // stop threads
    {
        ROS_DEBUG("[UDSClient::stop] stop threads");
        lock_guard<mutex> lock(mAccessMutex);
        if (!mStopFlag)
        {
            mStopFlag = true;
            for (auto &t : mThreadArray)
            {
                t.join();
            }
            mThreadArray.clear();
        }
        else
        {
            ROS_DEBUG("[UDSClient::stop] threads not running");
        }
    }
}

bool UDSClient::sendRawString(std::string &req)
{
    ROS_DEBUG("[UDSClient::sendRawString] put sig into sending queue");
    lock_guard<mutex> lock(mAccessMutex);
    mReqStrList.push_back(req);
    return true;
}

bool UDSClient::sendSig(Document &req)
{
    string json = RCMP::getJson(req);
    sendRawString(json);
    return true;
}

void UDSClient::threadFunc()
{
    ROS_DEBUG("[UDSClient::threadFunc] UDSClient thread start");

    while (!mStopFlag)
    {
        int epollfd = 0;
        UdsSession_t udss;

        // init env
        if (!initEnv(epollfd, udss))
        {
            ROS_ERROR("[UDSClient::threadFunc] init fail. sleep %d seconds",
                      INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
            continue;
        }

        // main routine
        try
        {
            int ret;
            list<string> reqList;

            udss.soc = udss.soc;
            udss.events = EPOLLIN;

            struct epoll_event events[MAX_CLIENT_COUNT];

            // add soc into epoll driver
            struct epoll_event event;
            event.events = udss.events;
            event.data.fd = udss.soc;
            if (epoll_ctl(epollfd, EPOLL_CTL_ADD, udss.soc, &event))
            {
                ROS_ERROR("[UDSClient::threadFunc] epoll add fail. Error[%d]: %s",
                          errno, strerror(errno));
                closeEnv(epollfd, udss);
                this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
                continue;
            }

            while (!mStopFlag)
            {
                ret = epoll_wait(epollfd, events, EPOLL_MAX_EVENTS, INTERVAL_EPOLL_RETRY);
                if (ret < 0)
                {
                    if (errno == EAGAIN || errno == EINTR)
                    {
                        this_thread::sleep_for(chrono::milliseconds(INTERVAL_EPOLL_RETRY));
                    }
                    else
                    {
                        ROS_ERROR("[UDSClient::threadFunc] epoll fail. Error[%d]: %s",
                                  errno, strerror(errno));
                        break;
                    }
                }
                else if (ret == 0)
                {
                    // epoll wait timeout
                }
                else
                {
                    assert(events[0].data.fd == udss.soc);

                    if (!onSoc(epollfd, events[0].events, udss))
                    {
                        ROS_DEBUG("[UDSClient::threadFunc] Remove soc[%d]", udss.soc);

                        // remove socket from epoll
                        epoll_ctl(epollfd, EPOLL_CTL_DEL, udss.soc, NULL);

                        // break thread routine loop
                        break;
                    }
                }

                // process request signalings
                if (!mReqStrList.empty() && mAccessMutex.try_lock())
                {
                    // swap buffer
                    reqList.swap(mReqStrList);
                    mAccessMutex.unlock();

                    for (auto &req : reqList)
                    {
                        if (!sendToBuf(udss.buffer, req))
                        {
                            ROS_ERROR("[UDSClient::threadFunc] send signaling to buffer fail, drop signalings.");
                            break;
                        }

                        setWritable(epollfd, udss, true);
                    }

                    reqList.clear();
                }
            }
        }
        catch (const exception &e)
        {
            ROS_ERROR_STREAM("[UDSClient::threadFunc] catch an exception." << e.what());
        }

        // close env
        closeEnv(epollfd, udss);

        if (!mStopFlag)
        {
            ROS_DEBUG("[UDSClient::threadFunc] sleep %d secnds and try again", INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(UDSClient::INTERVAL_CONNECT_RETRY));
        }
    }

    ROS_DEBUG("[UDSClient::threadFunc] UDSClient thread stop");
}

bool UDSClient::initEnv(int &epollfd, UdsSession_t &udss)
{
    // init epoll file descriptor
    ROS_DEBUG("[UDSClient::initEnv] init epoll file descriptor");
    if ((epollfd = epoll_create1(0)) < 0)
    {
        ROS_ERROR("[UDSClient::initEnv] Failed to create epoll. Error[%d]: %s",
                  errno, strerror(errno));
        closeEnv(epollfd, udss);
        return false;
    }

    // init Unix Domain Socket
    {
        ROS_DEBUG("[UDSClient::initEnv] init Unix Domain Socket: %s", mUdsUri.c_str());

        // create socket
        if ((udss.soc = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
        {
            ROS_ERROR("[UDSClient::initEnv] Fail to create socket. Error[%d]: %s",
                      errno, strerror(errno));
            closeEnv(epollfd, udss);
            return false;
        }

        // connect
        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, mUdsUri.c_str(), sizeof(addr.sun_path) - 1);
        if (connect(udss.soc, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        {
            ROS_ERROR("[UDSClient::initEnv] Fail to connect to UDS[%s]. Error[%d]: %s",
                      mUdsUri.c_str(), errno, strerror(errno));
            closeEnv(epollfd, udss);
            return false;
        }
    }

    return true;
}

void UDSClient::closeEnv(int &epollfd, UdsSession_t &udss)
{
    // close epoll file descriptor
    ROS_DEBUG("[UDSClient::closeEnv] close epoll file descriptor");
    if (epollfd)
    {
        if (close(epollfd))
        {
            ROS_ERROR("[UDSClient::closeEnv] Fail to close epoll. Error[%d]: %s",
                      errno, strerror(errno));
        }
        epollfd = 0;
    }

    // close Unix Domain Socket
    ROS_DEBUG("[UDSClient::closeEnv] close Unix Domain Socket");
    if (udss.soc)
    {
        if (close(udss.soc))
        {
            ROS_ERROR("[UDSClient::closeEnv] Fail to close Unix Domain Socket. Error[%d]: %s",
                      errno, strerror(errno));
        }
        udss.soc = 0;
    }
    udss.init();
}

bool UDSClient::onSoc(int epollfd, unsigned int socEvents, UdsSession_t &udss)
{
    if (socEvents & EPOLLIN)
    {
        // available for read
        if (!onRead(epollfd, udss))
        {
            ROS_DEBUG("[UDSClient::onSoc] read fail");
            return false;
        }
    }
    if (socEvents & EPOLLOUT)
    {
        // available for write
        if (!onWrite(epollfd, udss))
        {
            ROS_DEBUG("[UDSClient::onSoc] write fail");
            return false;
        }
    }
    if (socEvents & (EPOLLRDHUP | EPOLLHUP))
    {
        // socket has been closed
        ROS_DEBUG("[UDSClient::onSoc] client soc[%d] has been closed", udss.soc);
        return false;
    }
    if (socEvents & EPOLLERR)
    {
        // socket error
        ROS_ERROR("[UDSClient::onSoc] client soc[%d] error", udss.soc);
        return false;
    }

    return true;
}

bool UDSClient::onRead(int epollfd, UdsSession_t &udss)
{
    assert(udss.validate());

    // receive data from socket
    void *p;
    int len;

    std::tie(p, len) = udss.buffer.getRecvBuf();

    int nRet = recv(udss.soc, p, len, 0);
    if (nRet <= 0)
    {
        if (nRet == 0)
        {
            ROS_DEBUG("[UDSClient::onRead] session closed by peer(soc[%d])", udss.soc);
            return false;
        }
        else
        {
            ROS_ERROR("[UDSClient::onRead] soc[%d] read fail. Error[%d]: %s",
                      udss.soc, errno, strerror(errno));
            return false;
        }
    }

    // adjust end pos
    udss.buffer.incRecvEnd(nRet);
    assert(udss.validate());

    // get signaling from raw buffer
    Document doc;
    while (!udss.buffer.recvBufEmpty())
    {
        nRet = parseSig(udss.buffer, doc);
        if (SUCCESS != nRet)
        {
            if (nRet == NEED_MORE_DATA)
            {
                // need more data
                break;
            }

            ROS_ERROR("[UDSClient::onRead] soc[%d] get signaling fail: %d", udss.soc, nRet);
            return false;
        }
        else
        {
            // process signaling
            if (!mOnSigCallbak(udss.buffer, doc))
            {
                ROS_ERROR("[UDSClient::onRead] soc[%d] process signaling fail", udss.soc);
                return false;
            }
        }
    }

    // is recv buffer full
    if (udss.buffer.recvBufFull())
    {
        ROS_ERROR("[UDSClient::onRead] recv buf of soc[%d] full", udss.soc);
        return false;
    }

    // are there data in send buffer?
    if (!udss.buffer.sendBufEmpty())
    {
        return setWritable(epollfd, udss, true);
    }

    return true;
}

bool UDSClient::onWrite(int epollfd, UdsSession_t &udss)
{
    assert(udss.validate());

    void *p;
    int len;
    std::tie(p, len) = udss.buffer.getSendData();

    int nRet = send(udss.soc, p, len, 0);
    if (nRet < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return true;
        }

        ROS_ERROR("[UDSClient::onWrite] soc[%d] send fail. Error: %d - %s",
                  udss.soc, errno, strerror(errno));
        return false;
    }

    // ROS_DEBUG("[UDSClient::onWrite] soc[%d] send %d bytes", udss.soc, nRet);

    udss.buffer.incSendStart(nRet);
    if (udss.buffer.sendBufEmpty())
    {
        // stop send

        // ROS_DEBUG("[UDSClient::onWrite] soc[%d] stop send", udss.soc);

        return setWritable(epollfd, udss, false);
    }

    return true;
}

bool UDSClient::setWritable(int epollfd, UdsSession_t &udss, bool writable)
{
    bool update = false;
    struct epoll_event event;

    if (writable)
    {
        // set EPOLLOUT
        update = 0 == (udss.events & EPOLLOUT) ? udss.events |= EPOLLOUT, true : false;
    }
    else
    {
        // remove EPOLLOUT
        update = 1 == (udss.events & EPOLLOUT) ? udss.events &= ~EPOLLOUT, true : false;
    }

    if (update)
    {
        event.data.fd = udss.soc;
        event.events = udss.events;
        if (epoll_ctl(epollfd, EPOLL_CTL_MOD, udss.soc, &event))
        {
            ROS_ERROR("[UDSClient::setWritable] epoll_ctl for soc[%d] fail. Error[%d]: %s",
                      udss.soc, errno, strerror(errno));
            return false;
        }
    }

    return true;
}

int UDSClient::parseSig(BufType &buffer, Document &sig)
{
    char *p;
    int len;
    tie(p, len) = buffer.getRecvData();

    int nRet = RCMP::parse(p, len, sig);
    if (nRet != SUCCESS)
    {
        ROS_ERROR("[UDSClient::parseSig] RCMP parse fail.");
        return nRet;
    }

    // adjust buffer pos
    buffer.incRecvStart(len);

    return SUCCESS;
}

bool UDSClient::sendToBuf(BufType &buffer, Document &sig)
{
    string jsonSig = RCMP::getJson(sig);
    return sendToBuf(buffer, jsonSig);
}

bool UDSClient::sendToBuf(BufType &buffer, string &sig)
{
    // check empty send buffer size
    auto len = sig.length();
    char *p;
    int capacity;

    if ((tie(p, capacity) = buffer.getSendBuf(), capacity < len) &&
        (!buffer.defragSendBuf() ||                                 // defrag
         (tie(p, capacity) = buffer.getSendBuf(), capacity < len))) // try again
    {
        // send buffer full
        ROS_ERROR("[UDSClient::sendToBuf] send buffer fulll");
        return false;
    }

    int nRet = RCMP::fillFrame(p, capacity, sig.c_str(), len);
    if (nRet > 0)
    {
        buffer.incSendEnd(nRet);
        return true;
    }
    else
    {
        // buffer full
        ROS_ERROR("[UDSClient::sendToBuf] fill frame fail.");
        return false;
    }
}

} // namespace cli
} // namespace car
} // namespace roscar
