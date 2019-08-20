#include "udsClient.h"

#include <assert.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/epoll.h>
#include <chrono>

#include "ros/ros.h"
#include "roscar_common/error.h"
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
    : mUdsUri(),
      mEpollfd(0),
      mStopFlag(true),
      mOnSigCallbak(onSigCallbak)
{
    mUdsSession.init();
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

bool UDSClient::sendSig(std::string &sig)
{
    ROS_DEBUG("[UDSClient::sendSig] put sig into sending queue");
    lock_guard<mutex> lock(mAccessMutex);

    if (0 == mUdsSession.soc)
    {
        ROS_ERROR("[UDSClient::sendSig] send fail - connection not established.");
        return false;
    }

    // check empty send buffer size
    auto len = sig.length();
    char *p;
    int capacity;

    if ((tie(p, capacity) = mUdsSession.buffer.getSendBuf(), capacity < len) &&
        (!mUdsSession.buffer.defragSendBuf() ||                                 // defrag
         (tie(p, capacity) = mUdsSession.buffer.getSendBuf(), capacity < len))) // try again
    {
        // send buffer full
        ROS_ERROR("[UDSClient::sendSig] send buffer fulll");
        return false;
    }

    int nRet = RCMP::fillFrame(p, capacity, sig.c_str(), len);
    if (nRet <= 0)
    {
        // buffer full
        ROS_ERROR("[UDSClient::sendSig] fill frame fail.");
        return false;
    }
    mUdsSession.buffer.incSendEnd(nRet);

    return setSocWritable(mUdsSession, true);
}

void UDSClient::threadFunc()
{
    ROS_DEBUG("[UDSClient::threadFunc] UDSClient thread start");

    while (!mStopFlag)
    {
        // init env
        if (!initEnv())
        {
            ROS_ERROR("[UDSClient::threadFunc] init fail. sleep %d seconds",
                      INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
            continue;
        }

        // main routine
        try
        {
            struct epoll_event event;
            struct epoll_event events[EPOLL_MAX_EVENTS];

            // add soc into epoll driver
            mUdsSession.events = EPOLLIN;
            event.events = mUdsSession.events;
            event.data.fd = mUdsSession.soc;
            if (epoll_ctl(mEpollfd, EPOLL_CTL_ADD, event.data.fd, &event))
            {
                ROS_ERROR("[UDSClient::threadFunc] epoll add fail. Error[%d]: %s",
                          errno, strerror(errno));
                closeEnv();
                this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
                continue;
            }

            while (!mStopFlag)
            {
                int nRet = epoll_wait(mEpollfd, events, EPOLL_MAX_EVENTS, INTERVAL_EPOLL_RETRY);
                if (nRet < 0)
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
                else if (nRet == 0)
                {
                    // epoll wait timeout
                }
                else
                {
                    assert(events[0].data.fd == mUdsSession.soc);

                    // lock session data
                    lock_guard<mutex> lock(mAccessMutex);

                    if (!onSoc(events[0].events, mUdsSession))
                    {
                        ROS_DEBUG("[UDSClient::threadFunc] Remove soc[%d]", mUdsSession.soc);

                        // remove socket from epoll
                        epoll_ctl(mEpollfd, EPOLL_CTL_DEL, mUdsSession.soc, NULL);

                        // break thread routine loop
                        break;
                    }
                }
            }
        }
        catch (const exception &e)
        {
            ROS_ERROR_STREAM("[UDSClient::threadFunc] catch an exception." << e.what());
        }

        // close env
        closeEnv();

        if (!mStopFlag)
        {
            ROS_DEBUG("[UDSClient::threadFunc] sleep %d secnds and try again", INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(UDSClient::INTERVAL_CONNECT_RETRY));
        }
    }

    ROS_DEBUG("[UDSClient::threadFunc] UDSClient thread stop");
}

bool UDSClient::initEnv()
{
    // init epoll file descriptor
    ROS_DEBUG("[UDSClient::initEnv] init epoll file descriptor");
    if ((mEpollfd = epoll_create1(0)) < 0)
    {
        ROS_ERROR("[UDSClient::initEnv] Failed to create epoll. Error[%d]: %s",
                  errno, strerror(errno));
        closeEnv();
        return false;
    }

    // init Unix Domain Socket
    {
        ROS_DEBUG("[UDSClient::initEnv] init Unix Domain Socket: %s", mUdsUri.c_str());

        // create socket
        if ((mUdsSession.soc = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
        {
            ROS_ERROR("[UDSClient::initEnv] Fail to create socket. Error[%d]: %s",
                      errno, strerror(errno));
            closeEnv();
            return false;
        }

        // connect
        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, mUdsUri.c_str(), sizeof(addr.sun_path) - 1);
        if (connect(mUdsSession.soc, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        {
            ROS_ERROR("[UDSClient::initEnv] Fail to connect to UDS[%s]. Error[%d]: %s",
                      mUdsUri.c_str(), errno, strerror(errno));
            closeEnv();
            return false;
        }
    }

    return true;
}

void UDSClient::closeEnv()
{
    // close epoll file descriptor
    ROS_DEBUG("[UDSClient::closeEnv] close epoll file descriptor");
    if (mEpollfd)
    {
        if (close(mEpollfd))
        {
            ROS_ERROR("[UDSClient::closeEnv] Fail to close epoll. Error[%d]: %s",
                      errno, strerror(errno));
        }
        mEpollfd = 0;
    }

    // close Unix Domain Socket
    ROS_DEBUG("[UDSClient::closeEnv] close Unix Domain Socket");
    if (mUdsSession.soc)
    {
        if (close(mUdsSession.soc))
        {
            ROS_ERROR("[UDSClient::closeEnv] Fail to close Unix Domain Socket. Error[%d]: %s",
                      errno, strerror(errno));
        }
        mUdsSession.soc = 0;
    }
    mUdsSession.init();
}

bool UDSClient::onSoc(unsigned int socEvents, UdsSession_t &session)
{
    if (socEvents & EPOLLIN)
    {
        // available for read
        if (!onRead(session))
        {
            ROS_DEBUG("[UDSClient::onSoc] read fail");
            return false;
        }
    }
    if (socEvents & EPOLLOUT)
    {
        // available for write
        if (!onWrite(session))
        {
            ROS_DEBUG("[UDSClient::onSoc] write fail");
            return false;
        }
    }
    if (socEvents & (EPOLLRDHUP | EPOLLHUP))
    {
        // socket has been closed
        ROS_DEBUG("[UDSClient::onSoc] client soc[%d] has been closed", session.soc);
        return false;
    }
    if (socEvents & EPOLLERR)
    {
        // socket error
        ROS_ERROR("[UDSClient::onSoc] client soc[%d] error", session.soc);
        return false;
    }

    return true;
}

bool UDSClient::onRead(UdsSession_t &session)
{
    assert(session.validate());

    // receive data from socket
    void *p;
    int len;

    std::tie(p, len) = session.buffer.getRecvBuf();

    int nRet = recv(session.soc, p, len, 0);
    if (nRet < 0)
    {
        ROS_ERROR("[UDSClient::onRead] soc[%d] read fail. Error[%d]: %s",
                  session.soc, errno, strerror(errno));
        return false;
    }
    else if (nRet == 0)
    {
        ROS_DEBUG("[UDSClient::onRead] session closed by peer(soc[%d])", session.soc);
        return false;
    }

    // adjust end pos
    session.buffer.incRecvEnd(nRet);

    // get signaling from raw buffer
    Document doc;
    while (!session.buffer.recvBufEmpty())
    {
        nRet = parseSig(session.buffer, doc);
        if (SUCCESS != nRet)
        {
            if (nRet == NEED_MORE_DATA)
            {
                // need more data
                break;
            }

            ROS_ERROR("[UDSClient::onRead] soc[%d] get signaling fail: %d", session.soc, nRet);
            return false;
        }
        else
        {
            // process signaling
            if (!mOnSigCallbak(session.buffer, doc))
            {
                ROS_ERROR("[UDSClient::onRead] soc[%d] process signaling fail", session.soc);
                return false;
            }
        }
    }

    // is recv buffer full
    if (session.buffer.recvBufFull())
    {
        ROS_ERROR("[UDSClient::onRead] recv buf of soc[%d] full", session.soc);
        return false;
    }

    // are there data in send buffer?
    if (!session.buffer.sendBufEmpty())
    {
        return setSocWritable(session, true);
    }

    return true;
}

bool UDSClient::onWrite(UdsSession_t &session)
{
    assert(session.validate());

    void *p;
    int len;
    std::tie(p, len) = session.buffer.getSendData();

    int nRet = send(session.soc, p, len, 0);
    if (nRet < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return true;
        }

        ROS_ERROR("[UDSClient::onWrite] soc[%d] send fail. Error: %d - %s",
                  session.soc, errno, strerror(errno));
        return false;
    }

    // ROS_DEBUG("[UDSClient::onWrite] soc[%d] send %d bytes", session.soc, nRet);

    session.buffer.incSendStart(nRet);
    if (session.buffer.sendBufEmpty())
    {
        // stop send

        // ROS_DEBUG("[UDSClient::onWrite] soc[%d] stop send", session.soc);

        return setSocWritable(session, false);
    }

    return true;
}

bool UDSClient::setSocWritable(UdsSession_t &session, bool writable)
{
    bool update = false;
    struct epoll_event event;

    if (writable)
    {
        // set EPOLLOUT
        update = 0 == (session.events & EPOLLOUT) ? session.events |= EPOLLOUT, true : false;
    }
    else
    {
        // remove EPOLLOUT
        update = 1 == (session.events & EPOLLOUT) ? session.events &= ~EPOLLOUT, true : false;
    }

    if (update)
    {
        event.data.fd = session.soc;
        event.events = session.events;
        if (epoll_ctl(mEpollfd, EPOLL_CTL_MOD, session.soc, &event))
        {
            ROS_ERROR("[UDSClient::setSocWritable] epoll_ctl for soc[%d] fail. Error[%d]: %s",
                      session.soc, errno, strerror(errno));
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

} // namespace cli
} // namespace car
} // namespace roscar
