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

UDSClient::UDSClient(FUNC_ONSIGLAING cb_onSig)
    : mUdsUri(), mStopFlag(true), mCb_OnSig(cb_onSig)
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

bool UDSClient::sendSig(rapidjson::Document &req)
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
        int udsSoc = 0;

        // init env
        if (!initEnv(epollfd, udsSoc))
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
            SESSION_t sess;
            list<string> reqList;

            sess.soc = udsSoc;
            sess.events = EPOLLIN;

            struct epoll_event events[MAX_CLIENT_COUNT];

            // add soc into epoll driver
            struct epoll_event event;
            event.events = sess.events;
            event.data.fd = sess.soc;
            if (epoll_ctl(epollfd, EPOLL_CTL_ADD, udsSoc, &event))
            {
                ROS_ERROR("[UDSClient::threadFunc] epoll add fail. Error[%d]: %s",
                          errno, strerror(errno));
                closeEnv(epollfd, udsSoc);
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
                    assert(events[0].data.fd == udsSoc);

                    if (!onSoc(epollfd, events[0].events, sess))
                    {
                        ROS_DEBUG("[UDSClient::threadFunc] Remove socket[%d]", udsSoc);

                        // remove socket from epoll
                        epoll_ctl(epollfd, EPOLL_CTL_DEL, udsSoc, NULL);

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
                        if (!sendToBuf(sess, req))
                        {
                            ROS_ERROR("[UDSClient::threadFunc] send signaling to buffer fail, drop signalings.");
                            break;
                        }

                        setWriteFlag(epollfd, sess, true);
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
        closeEnv(epollfd, udsSoc);

        if (!mStopFlag)
        {
            ROS_DEBUG("[UDSClient::threadFunc] sleep %d secnds and try again", INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(UDSClient::INTERVAL_CONNECT_RETRY));
        }
    }

    ROS_DEBUG("[UDSClient::threadFunc] UDSClient thread stop");
}

bool UDSClient::initEnv(int & epollfd, int & soc)
{
    // init epoll file descriptor
    ROS_DEBUG("[UDSClient::initEnv] init epoll file descriptor");
    if ((epollfd = epoll_create1(0)) < 0)
    {
        ROS_ERROR("[UDSClient::initEnv] Failed to create epoll. Error[%d]: %s",
                  errno, strerror(errno));
        closeEnv(epollfd, soc);
        return false;
    }

    // init Unix Domain Socket
    {
        ROS_DEBUG("[UDSClient::initEnv] init Unix Domain Socket: %s", mUdsUri.c_str());

        // create socket
        if ((soc = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
        {
            ROS_ERROR("[UDSClient::initEnv] Fail to create socket. Error[%d]: %s",
                      errno, strerror(errno));
            closeEnv(epollfd, soc);
            return false;
        }

        // connect
        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, mUdsUri.c_str(), sizeof(addr.sun_path) - 1);
        if (connect(soc, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        {
            ROS_ERROR("[UDSClient::initEnv] Fail to connect to UDS[%s]. Error[%d]: %s",
                      mUdsUri.c_str(), errno, strerror(errno));
            closeEnv(epollfd, soc);
            return false;
        }
    }

    return true;
}

void UDSClient::closeEnv(int & epollfd, int & soc)
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
    if (soc)
    {
        if (close(soc))
        {
            ROS_ERROR("[UDSClient::closeEnv] Fail to close Unix Domain Socket. Error[%d]: %s",
                      errno, strerror(errno));
        }
        soc = 0;
    }
}

bool UDSClient::onSoc(int epollfd, unsigned int socEvents, SESSION_t &sess)
{
    if (socEvents & EPOLLIN)
    {
        // available for read
        if (!onRead(epollfd, sess))
        {
            ROS_DEBUG("[UDSClient::onSoc] read fail");
            return false;
        }
    }
    if (socEvents & EPOLLOUT)
    {
        // available for write
        if (!onWrite(epollfd, sess))
        {
            ROS_DEBUG("[UDSClient::onSoc] write fail");
            return false;
        }
    }
    if (socEvents & (EPOLLRDHUP | EPOLLHUP))
    {
        // socket has been closed
        ROS_DEBUG("[UDSClient::onSoc] client socket[%d] has been closed", sess.soc);
        return false;
    }
    if (socEvents & EPOLLERR)
    {
        // socket error
        ROS_ERROR("[UDSClient::onSoc] client socket[%d] error", sess.soc);
        return false;
    }

    return true;
}

bool UDSClient::onRead(int epollfd, SESSION_t &sess)
{
    assert(sess.recvBufPos >= 0);
    assert(sess.recvBufPos <= sess.recvBufEnd);
    assert(sess.recvBufEnd < RECV_BUFFER_CAPACITY);

    // receive data from socket
    int bufSize = RECV_BUFFER_CAPACITY - sess.recvBufEnd;
    int nRet = recv(sess.soc,
                    sess.recvBuf + sess.recvBufEnd,
                    bufSize,
                    0);
    if (nRet <= 0)
    {
        if (nRet == 0)
        {
            ROS_DEBUG("[UDSClient::onRead] session closed by peer(soc[%d])", sess.soc);
            return false;
        }
        else
        {
            ROS_ERROR("[UDSClient::onRead] client[%d] read fail. Error[%d]: %s",
                      sess.soc, errno, strerror(errno));
            return false;
        }
    }
    sess.recvBufEnd += nRet;

    // get signaling from raw buffer
    Document doc;
    while (!sess.isRecvBufEmpty())
    {
        nRet = parseRawBuffer(sess, doc);
        if (SUCCESS != nRet)
        {
            if (nRet == NEED_MORE_DATA)
            {
                // need more data
                break;
            }

            ROS_ERROR("[UDSClient::onRead] socket[%d] get signaling fail: %d", sess.soc, nRet);
            return false;
        }
        else
        {
            // process signaling
            if (!mCb_OnSig(sess, doc))
            {
                ROS_ERROR("[UDSClient::onRead] socket[%d] process signaling fail", sess.soc);
                return false;
            }
        }
    }

    // adjust recv buffer pos variables
    if (sess.isRecvBufFull())
    {
        // defrag recv buffer
        if (sess.defragRecvBuf())
        {
            return true;
        }
        else
        {
            ROS_ERROR("[UDSClient::onRead] defrage recv buffer for soc[%d] fail", sess.soc);
            return false;
        }
    }

    // are there data in send buffer?
    if (sess.sendBufPos ^ sess.sendBufEnd)
    {
        return setWriteFlag(epollfd, sess, true);
    }

    return true;
}

bool UDSClient::onWrite(int epollfd, SESSION_t &sess)
{
    assert(sess.recvBufPos >= 0);
    assert(sess.recvBufPos <= sess.recvBufEnd);
    assert(sess.recvBufEnd < RECV_BUFFER_CAPACITY);

    int nRet = send(sess.soc,
                    &sess.sendBuf[sess.sendBufPos],
                    sess.sendBufEnd - sess.sendBufPos,
                    0);
    if (nRet < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return true;
        }

        ROS_ERROR("[UDSClient::onWrite] socket[%d] write fail. Error[%d]: %s",
                  sess.soc, errno, strerror(errno));
        return false;
    }

    // ROS_DEBUG("[UDSClient::onWrite] send %d bytes", nRet);

    sess.sendBufPos += nRet;
    if (sess.recvBufPos == sess.recvBufEnd)
    {
        // no more data to send
        sess.recvBufPos = sess.recvBufEnd = 0;

        // remove epoll write event
        return setWriteFlag(epollfd, sess, false);
    }

    return true;
}

int UDSClient::parseRawBuffer(SESSION_t &sess, rapidjson::Document &sig)
{
    char *rawBuf = sess.recvBuf + sess.recvBufPos;
    int len = sess.recvBufEnd - sess.recvBufPos;

    int nRet = RCMP::parse(rawBuf, len, sig);
    if (nRet != SUCCESS)
    {
        return nRet;
    }

    // adjust buffer pos
    sess.recvBufPos += len;
    if (sess.recvBufPos == sess.recvBufEnd)
    {
        sess.recvBufPos = sess.recvBufEnd = 0;
    }

    return SUCCESS;
}

bool UDSClient::sendToBuf(SESSION_t &sess, rapidjson::Document &sig)
{
    // get corresponding json string
    string jsonSig = RCMP::getJson(sig);

    return sendToBuf(sess, jsonSig);
}

bool UDSClient::sendToBuf(SESSION_t &sess, string &sig)
{
    // check empty send buffer size
    auto len = sig.length();
    if (RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd < len)
    {
        if (sess.sendBufPos == 0 ||
            (RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd + sess.sendBufPos < len))
        {
            // send buffer full
            ROS_ERROR("[UDSClient::sendToBuf] send buffer fulll(soc: [%d]", sess.soc);
            return false;
        }

        // defrag buffer
        int bufLen = sess.sendBufEnd - sess.sendBufPos;
        memmove(sess.sendBuf, sess.sendBuf + sess.sendBufPos, bufLen);
        sess.sendBufPos = 0;
        sess.sendBufEnd = bufLen;
    }

    int nRet = RCMP::fillFrame(sess.sendBuf + sess.sendBufEnd,
                               RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd,
                               sig.c_str(),
                               len);
    if (nRet > 0)
    {
        sess.sendBufEnd += nRet;
        return true;
    }
    else
    {
        // send buffer full
        ROS_ERROR("[UDSClient::sendToBuf] fill frame fail.");
        return false;
    }

    return true;
}

bool UDSClient::setWriteFlag(int epollfd, SESSION_t &sess, bool writeFlag)
{
    // are there data in send buffer?
    if (writeFlag)
    {
        // set epoll write flag for this socket
        if (!(sess.events & EPOLLOUT))
        {
            // ROS_DEBUG("[UDSClient::setWriteFlag] set send flag");
            sess.events |= EPOLLOUT;

            struct epoll_event event;
            event.data.fd = sess.soc;
            event.events = sess.events;
            if (epoll_ctl(epollfd, EPOLL_CTL_MOD, sess.soc, &event))
            {
                ROS_ERROR("[UDSClient::setWriteFlag] Failed to add poll-out event for client[%d]. Error[%d]: %s",
                          sess.soc, errno, strerror(errno));
                return false;
            }
        }
    }
    else
    {
        // remove epoll write flag from this socket
        if (sess.events & EPOLLOUT)
        {
            // ROS_DEBUG("[UDSClient::setWriteFlag] remove send flag");
            sess.events &= ~EPOLLOUT;

            struct epoll_event event;
            event.data.fd = sess.soc;
            event.events = sess.events;
            if (epoll_ctl(epollfd, EPOLL_CTL_MOD, sess.soc, &event))
            {
                ROS_ERROR("[UDSClient::setWriteFlag] Failed to remove poll-out event for client[%d]. Error[%d]: %s",
                          sess.soc, errno, strerror(errno));
                return false;
            }
        }
    }

    return true;
}

} // namespace cli
} // namespace car
} // namespace roscar
