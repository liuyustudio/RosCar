#include "uds.h"

#include <assert.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/epoll.h>

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

UDS::UDS(FUNC_ONSIGLAING cb_onSig)
    : mStopUDS(false), mCb_OnSig(cb_onSig)
{
}

UDS::~UDS()
{
    stop();
}

bool UDS::start(const char *udsUri)
{
    ROS_DEBUG("Debug: start UDS");

    // init epoll file descriptor
    if ((mEpollfd = epoll_create1(0)) < 0)
    {
        ROS_ERROR("Err: Failed to create epoll. Error[%d]: %s",
                  errno, strerror(errno));
        stop();
        return false;
    }

    // init Unix Domain Socket
    {
        ROS_DEBUG("Debug: init Unix Domain Socket: %s", udsUri);
        if ((mUdsSoc = socket(AF_UNIX, SOCK_STREAM, 0)) <= 0)
        {
            ROS_ERROR("Err: Fail to create socket. Error[%d]: %s",
                      errno, strerror(errno));
            stop();
            return false;
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, udsUri, sizeof(addr.sun_path) - 1);
        unlink(udsUri);

        if (bind(mUdsSoc, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        {
            ROS_ERROR("Err: Fail to bind[%s]. Error[%d]: %s",
                      udsUri, errno, strerror(errno));
            stop();
            return false;
        }

        if (listen(mUdsSoc, SOMAXCONN) == -1)
        {
            ROS_ERROR("Err: Listen fail. Error[%d]: %s",
                      errno, strerror(errno));
            stop();
            return false;
        }

        // start thread
        mThreadArray.emplace_back(&UDS::threadFunc, this);
    }

    ROS_DEBUG("Debug: UDS started");

    return true;
}

void UDS::stop()
{
    ROS_DEBUG("Debug: stop UDS");

    // stop threads
    mStopUDS = true;
    for (auto &t : mThreadArray)
    {
        t.join();
    }

    // close epoll file descriptor
    if (mEpollfd)
    {
        close(mEpollfd);
        mEpollfd = 0;
    }

    // close Unix Domain Socket
    if (mUdsSoc)
    {
        close(mUdsSoc);
        mUdsSoc = 0;
    }

    ROS_DEBUG("Debug: UDS stopped");
}

void UDS::threadFunc()
{
    ROS_DEBUG("Debug: init UDS thread");
    map<int, SESSION_t> soc2Sess;

    int soc;
    int ret;
    SESSION_t sess;

    struct epoll_event event;
    struct epoll_event events[MAX_CLIENT_COUNT];

    event.events = EPOLLIN;
    event.data.fd = mUdsSoc;
    if (epoll_ctl(mEpollfd, EPOLL_CTL_ADD, mUdsSoc, &event))
    {
        ROS_ERROR("Err: Failed to add DUS fd to epoll. Error[%d]: %s",
                  errno, strerror(errno));
        return;
    }

    ROS_DEBUG("Debug: UDS thread start");
    while (!mStopUDS)
    {
        ret = epoll_wait(mEpollfd, events, EPOLL_MAX_EVENTS, EPOLL_TIMEOUT);
        if (ret < 0)
        {
            if (errno == EAGAIN || errno == EINTR)
            {
                usleep(EPOLL_RETRY_INTERVAL);
            }
            else
            {
                ROS_ERROR("Err: epoll fail. Error[%d]: %s", errno, strerror(errno));

                // break thread routine loop
                break;
            }
        }

        for (int i = 0; i < ret; ++i)
        {
            if (events[i].data.fd == mUdsSoc)
            {
                // UDS server socket

                // accept client
                if ((soc = accept(mUdsSoc, NULL, NULL)) == -1)
                {
                    ROS_ERROR("Err: accept fail. Error[%d]: %s", errno, strerror(errno));
                    continue;
                }

                // add client soc into epoll
                event.data.fd = soc;
                event.events = EPOLLIN;
                if (epoll_ctl(mEpollfd, EPOLL_CTL_ADD, soc, &event))
                {
                    ROS_ERROR("Err: Failed to add client fd to epoll. Error[%d]: %s",
                              errno, strerror(errno));
                    close(soc);
                    continue;
                }

                // bind client socket with session data
                sess.soc = event.data.fd;
                sess.events = event.events;
                soc2Sess[soc] = sess;

                ROS_DEBUG("Debug: Accept socket[%d]", soc);
            }
            else
            {
                // client socket
                soc = events[i].data.fd;
                SESSION_t &sess = soc2Sess[soc];
                sess.events = events[i].events;

                if (!onSession(sess))
                {
                    ROS_DEBUG("Debug: Remove socket[%d]", soc);

                    // unbind client socket and session
                    soc2Sess.erase(soc);
                    // remove socket from epoll
                    epoll_ctl(mEpollfd, EPOLL_CTL_DEL, soc, NULL);
                }
            }
        }
    }

    ROS_DEBUG("Debug: UDS thread stop");
}

bool UDS::onSession(SESSION_t &sess)
{
    if (sess.events & EPOLLIN)
    {
        // available for read
        return onRead(sess);
    }
    if (sess.events & EPOLLOUT)
    {
        // available for write
        return onWrite(sess);
    }
    if (sess.events & (EPOLLRDHUP | EPOLLHUP))
    {
        // socket has been closed
        ROS_DEBUG("Debug: socket[%d] has been closed", sess.soc);
        return false;
    }
    if (sess.events & EPOLLERR)
    {
        // socket has been closed
        ROS_ERROR("Err: socket[%d]", sess.soc);
        return false;
    }
}

bool UDS::onRead(SESSION_t &sess)
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
        ROS_ERROR("Err: client[%d] read fail. Error[%d]: %s",
                  sess.soc, errno, strerror(errno));
        return false;
    }
    else if (nRet == 0)
    {
        ROS_INFO("Info: client[%d] session was broken", sess.soc);
        return false;
    }
    sess.recvBufEnd += nRet;

    // parse signaling from raw buffer
    Document doc;
    if (!parseSig(sess, doc))
    {
        ROS_ERROR("Err: socket[%d] parse signaling fail", sess.soc);
        return false;
    }

    // process signaling
    if (!mCb_OnSig(sess, doc))
    {
        ROS_ERROR("Err: socket[%d] process signaling fail", sess.soc);
        return false;
    }

    // adjust recv buffer pos variables
    if (sess.recvBufEnd == RECV_BUFFER_CAPACITY)
    {
        if (sess.recvBufPos == 0)
        {
            ROS_ERROR("Err: recv buffer of socket[%d] full", sess.soc);
            return false;
        }
        else
        {
            memmove(sess.recvBuf, sess.recvBuf + sess.recvBufPos, sess.recvBufPos);
            sess.recvBufEnd -= sess.recvBufPos;
            sess.recvBufPos = 0;
        }
    }

    // are there data in send buffer?
    if (sess.sendBufPos ^ sess.sendBufEnd)
    {
        // set epoll write event for this socket
        if (0 == sess.events & EPOLLOUT)
        {
            sess.events |= EPOLLOUT;

            struct epoll_event event;
            event.data.fd = sess.soc;
            event.events = sess.events;
            if (epoll_ctl(mEpollfd, EPOLL_CTL_MOD, sess.soc, &event))
            {
                ROS_ERROR("Err: Failed to add poll-out event for client[%d]. Error[%d]: %s",
                          sess.soc, errno, strerror(errno));
                return false;
            }
        }
    }
}

bool UDS::onWrite(SESSION_t &sess)
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

        ROS_ERROR("Err: socket[%d] write fail. Error[%d]: %s", sess.soc, errno, strerror(errno));
        return false;
    }

    sess.sendBufPos += nRet;
    if (sess.recvBufPos == sess.recvBufEnd)
    {
        // no more data to send
        sess.recvBufPos = sess.recvBufEnd = 0;

        // remove epoll write event
        epoll_event event;
        event.data.fd = sess.soc;
        event.events = sess.events & (~EPOLLOUT);
        if (epoll_ctl(mEpollfd, EPOLL_CTL_MOD, sess.soc, &event))
        {
            ROS_ERROR("Err: Failed to modify socket[%d] epoll event fail. Error[%d]: %s",
                      sess.soc, errno, strerror(errno));
            return false;
        }
    }
}

bool UDS::parseSig(SESSION_t &sess, rapidjson::Document &doc)
{
    char *rawBuf = sess.recvBuf + sess.recvBufPos;
    int len = sess.recvBufEnd - sess.recvBufPos;

    int nRet = RCMP::parse(rawBuf, len, doc);
    if (nRet != SUCCESS)
    {
        if (nRet == NEED_MORE_DATA)
        {
            // need more data
            return true;
        }
        else
        {
            ROS_DEBUG("Debug: parse signaling fail: %d", nRet);
            return false;
        }
    }

    // adjust buffer pos
    sess.recvBufPos += len;
    if (sess.recvBufPos == sess.recvBufEnd)
    {
        sess.recvBufPos = sess.recvBufEnd = 0;
    }
}

} // namespace interface
} // namespace car
} // namespace roscar
