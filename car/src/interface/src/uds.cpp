#include "uds.h"

#include <assert.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/epoll.h>
#include <chrono>
#include <exception>

#include "ros/ros.h"
#include "roscar_common/error.h"

using namespace std;
using namespace rapidjson;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace interface
{

const int UDS::INTERVAL_EPOLL_RETRY = 100;
const int UDS::INTERVAL_CONNECT_RETRY = 7;

UDS::UDS(FUNC_ONSIGLAING cb_onSig)
    : mUdsUri(), mStopFlag(true), mCb_OnSig(cb_onSig)
{
}

UDS::~UDS()
{
    stop();
}

bool UDS::start(const char *udsUri)
{
    mUdsUri = udsUri;

    // start thread
    {
        ROS_DEBUG("[UDS::start] start thread");
        lock_guard<mutex> lock(mAccessMutex);
        if (!mStopFlag)
        {
            ROS_ERROR("[UDS::start] stop thread first");
            return false;
        }

        mStopFlag = false;
        mThreadArray.emplace_back(&UDS::threadFunc, this);
    }

    return true;
}

void UDS::stop()
{
    ROS_DEBUG("[UDS::stop] stop UDS");

    // stop threads
    {
        ROS_DEBUG("[UDS::stop] stop threads");
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
            ROS_DEBUG("[UDS::stop] threads not running");
        }
    }
}

void UDS::threadFunc()
{
    ROS_DEBUG("[UDS::threadFunc] UDS thread start");
    while (!mStopFlag)
    {
        int epollfd = 0;
        int udsSoc = 0;

        // init env
        if (!initEnv(epollfd, udsSoc))
        {
            ROS_ERROR("[UDS::threadFunc] init fail. sleep %d seconds",
                      INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
            continue;
        }

        // main routine
        try
        {
            map<int, SESSION_t> soc2Sess;

            int soc;
            int ret;
            SESSION_t sess;

            struct epoll_event event;
            struct epoll_event events[MAX_CLIENT_COUNT];

            event.events = EPOLLIN;
            event.data.fd = udsSoc;
            if (epoll_ctl(epollfd, EPOLL_CTL_ADD, udsSoc, &event))
            {
                ROS_ERROR("[UDS::threadFunc] epoll add fail. Error[%d]: %s",
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
                        ROS_ERROR("[UDS::threadFunc] epoll fail. Error[%d]: %s",
                                  errno, strerror(errno));
                        break;
                    }
                }
                else if (ret == 0)
                {
                    // timeout
                }
                else
                {
                    for (int i = 0; i < ret; ++i)
                    {
                        if (events[i].data.fd == udsSoc)
                        {
                            // UDS server socket

                            // accept client
                            if ((soc = accept(udsSoc, NULL, NULL)) == -1)
                            {
                                ROS_ERROR("[UDS::threadFunc] accept fail. Error[%d]: %s", errno, strerror(errno));
                                continue;
                            }

                            // add client soc into epoll
                            event.data.fd = soc;
                            event.events = EPOLLIN;
                            if (epoll_ctl(epollfd, EPOLL_CTL_ADD, soc, &event))
                            {
                                ROS_ERROR("[UDS::threadFunc] Failed to add client fd to epoll. Error[%d]: %s",
                                          errno, strerror(errno));
                                close(soc);
                                continue;
                            }

                            // bind client socket with session data
                            sess.soc = event.data.fd;
                            sess.events = event.events;
                            soc2Sess[soc] = sess;

                            ROS_DEBUG("[UDS::threadFunc] Accept client[%d]", soc);
                        }
                        else
                        {
                            // client socket
                            soc = events[i].data.fd;
                            SESSION_t &sess = soc2Sess[soc];

                            if (!onSession(epollfd, events[i].events, sess))
                            {
                                ROS_DEBUG("[UDS::threadFunc] Remove socket[%d]", soc);

                                // unbind client socket and session
                                soc2Sess.erase(soc);
                                // remove socket from epoll
                                epoll_ctl(epollfd, EPOLL_CTL_DEL, soc, NULL);
                            }
                        }
                    }
                }
            }
        }
        catch (const exception &e)
        {
            ROS_ERROR_STREAM("[UDS::threadFunc] catch an exception." << e.what());
        }

        // close env
        closeEnv(epollfd, udsSoc);

        if (!mStopFlag)
        {
            ROS_DEBUG("[UDS::threadFunc] sleep %d secnds and try again", INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
        }
    }

    ROS_DEBUG("[UDS::threadFunc] UDS thread stop");
}

bool UDS::initEnv(int &epollfd, int &soc)
{
    // create epoll
    ROS_DEBUG("[UDS::initEnv] create epoll");
    if ((epollfd = epoll_create1(0)) < 0)
    {
        ROS_ERROR("[UDS::initEnv] Failed to create epoll. Error[%d]: %s",
                  errno, strerror(errno));
        closeEnv(epollfd, soc);
        return false;
    }

    // init Unix Domain Socket
    {
        ROS_DEBUG("[UDS::initEnv] init Unix Domain Socket: %s", mUdsUri.c_str());
        if ((soc = socket(AF_UNIX, SOCK_STREAM, 0)) <= 0)
        {
            ROS_ERROR("[UDS::initEnv] Fail to create socket. Error[%d]: %s",
                      errno, strerror(errno));
            closeEnv(epollfd, soc);
            return false;
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, mUdsUri.c_str(), sizeof(addr.sun_path) - 1);
        unlink(mUdsUri.c_str());

        if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        {
            ROS_ERROR("[UDS::initEnv] Fail to bind[%s]. Error[%d]: %s",
                      mUdsUri.c_str(), errno, strerror(errno));
            closeEnv(epollfd, soc);
            return false;
        }

        if (listen(soc, SOMAXCONN) == -1)
        {
            ROS_ERROR("[UDS::initEnv] Listen fail. Error[%d]: %s",
                      errno, strerror(errno));
            closeEnv(epollfd, soc);
            return false;
        }
    }

    return true;
}

void UDS::closeEnv(int & epollfd, int & soc)
{
    // close epoll file descriptor
    ROS_DEBUG("[UDS::closeEnv] close epoll file descriptor");
    if (epollfd)
    {
        if (close(epollfd))
        {
            ROS_ERROR("[UDS::closeEnv] Fail to close epoll. Error[%d]: %s",
                      errno, strerror(errno));
        }
        epollfd = 0;
    }

    // close Unix Domain Socket
    ROS_DEBUG("[UDS::closeEnv] close Unix Domain Socket");
    if (soc)
    {
        if (close(soc))
        {
            ROS_ERROR("[UDS::closeEnv] Fail to close Unix Domain Socket. Error[%d]: %s",
                      errno, strerror(errno));
        }
        soc = 0;
    }
}

bool UDS::onSession(int epollfd, unsigned int socEvents, SESSION_t &sess)
{
    if (socEvents & EPOLLIN)
    {
        // available for read
        if (!onRead(epollfd, sess))
        {
            return false;
        }
    }
    if (socEvents & EPOLLOUT)
    {
        // available for write
        if (!onWrite(epollfd, sess))
        {
            return false;
        }
    }
    if (socEvents & (EPOLLRDHUP | EPOLLHUP))
    {
        // socket has been closed
        ROS_DEBUG("[UDS::onSession] socket[%d] has been closed", sess.soc);
        return false;
    }
    if (socEvents & EPOLLERR)
    {
        // socket has been closed
        ROS_ERROR("[UDS::onSession] socket[%d]", sess.soc);
        return false;
    }

    return true;
}

bool UDS::onRead(int epollfd, SESSION_t &sess)
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
    if (nRet < 0)
    {
        ROS_ERROR("[UDS::onRead] client[%d] read fail. Error[%d]: %s",
                  sess.soc, errno, strerror(errno));
        return false;
    }
    else if (nRet == 0)
    {
        ROS_DEBUG("[UDS::onRead] client[%d] session closed by peer", sess.soc);
        return false;
    }
    sess.recvBufEnd += nRet;

    // parse signaling from raw buffer
    Document doc;
    while (!sess.isRecvBufEmpty())
    {
        nRet = parseSig(sess, doc);
        if (nRet != SUCCESS)
        {
            if (nRet == NEED_MORE_DATA)
            {
                // need more data
                break;
            }
            else
            {
                ROS_ERROR("[UDS::onRead] socket[%d] parse signaling fail", sess.soc);
                return false;
            }
        }

        // process signaling
        if (!mCb_OnSig(sess, doc))
        {
            ROS_ERROR("[UDS::onRead] socket[%d] process signaling fail", sess.soc);
            return false;
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
            ROS_ERROR("[UDS::onRead] defrage recv buffer for soc[%d] fail", sess.soc);
            return false;
        }
    }

    // are there data in send buffer?
    if (!sess.isSendBufEmpty())
    {
        // set epoll write event for this socket
        if (0 == (sess.events & EPOLLOUT))
        {
            sess.events |= EPOLLOUT;

            struct epoll_event event;
            event.data.fd = sess.soc;
            event.events = sess.events;
            if (epoll_ctl(epollfd, EPOLL_CTL_MOD, sess.soc, &event))
            {
                ROS_ERROR("[UDS::onRead] Failed to add poll-out event for client[%d]. Error[%d]: %s",
                          sess.soc, errno, strerror(errno));
                return false;
            }
        }
    }

    return true;
}

bool UDS::onWrite(int epollfd, SESSION_t &sess)
{
    assert(sess.recvBufPos >= 0);
    assert(sess.recvBufPos <= sess.recvBufEnd);
    assert(sess.recvBufEnd < RECV_BUFFER_CAPACITY);

    int nRet = send(sess.soc, &sess.sendBuf[sess.sendBufPos], sess.sendBufEnd - sess.sendBufPos, 0);
    if (nRet < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return true;
        }

        ROS_ERROR("[UDS::onWrite] socket[%d] write fail. Error[%d]: %s", sess.soc, errno, strerror(errno));
        return false;
    }

    sess.sendBufPos += nRet;
    if (sess.recvBufPos == sess.recvBufEnd)
    {
        // no more data to send
        sess.recvBufPos = sess.recvBufEnd = 0;

        // remove epoll write event
        sess.events &= (~EPOLLOUT);
        epoll_event event;
        event.data.fd = sess.soc;
        event.events = sess.events;
        if (epoll_ctl(epollfd, EPOLL_CTL_MOD, sess.soc, &event))
        {
            ROS_ERROR("[UDS::onWrite] Failed to modify socket[%d] epoll event fail. Error[%d]: %s",
                      sess.soc, errno, strerror(errno));
            return false;
        }
    }

    return true;
}

int UDS::parseSig(SESSION_t &sess, rapidjson::Document &doc)
{
    char *rawBuf = sess.recvBuf + sess.recvBufPos;
    int len = sess.recvBufEnd - sess.recvBufPos;

    int nRet = RCMP::parse(rawBuf, len, doc);
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

} // namespace interface
} // namespace car
} // namespace roscar
