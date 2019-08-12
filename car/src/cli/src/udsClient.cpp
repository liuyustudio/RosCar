#include "udsClient.h"

#include <assert.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/epoll.h>

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

UDSClient::UDSClient(FUNC_ONSIGLAING cb_onSig)
    : mStopUDS(false), mCb_OnSig(cb_onSig)
{
}

UDSClient::~UDSClient()
{
    stop();
}

bool UDSClient::start(const char *udsUri)
{
    // init epoll file descriptor
    if ((mEpollfd = epoll_create1(0)) < 0)
    {
        ROS_ERROR("Err: Failed to create epoll. Error[%d]: %s\n",
                  errno, strerror(errno));
        return false;
    }

    // init Unix Domain Socket
    {
        // create socket
        ROS_DEBUG("Debug: connect to Unix Domain Socket: %s", udsUri);
        if ((mUdsSoc = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
        {
            ROS_ERROR("Err: Fail to create socket. Error[%d]: %s\n",
                      errno, strerror(errno));
            return false;
        }

        // connect
        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, udsUri, sizeof(addr.sun_path) - 1);
        if (connect(mUdsSoc, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        {
            ROS_ERROR("Err: Fail to connect to UDS[%s]. Error[%d]: %s\n",
                      udsUri, errno, strerror(errno));
            close(mUdsSoc);
            mUdsSoc = 0;
            return false;
        }
    }

    // start threads
    mThreadArray.emplace_back(&UDSClient::threadFunc, this);

    return true;
}

void UDSClient::stop()
{
    // stop threads
    mStopUDS = true;
    for (auto &t : mThreadArray)
    {
        t.join();
    }

    // close epoll file descriptor
    if (mEpollfd)
    {
        if (close(mEpollfd))
        {
            ROS_ERROR("Err: Fail to close epoll. Error[%d]: %s\n",
                      errno, strerror(errno));
        }
        mEpollfd = 0;
    }

    // close Unix Domain Socket
    if (mUdsSoc)
    {
        if (close(mUdsSoc))
        {
            ROS_ERROR("Err: Fail to close Unix Domain Socket. Error[%d]: %s\n",
                      errno, strerror(errno));
        }
        mUdsSoc = 0;
    }
}

bool UDSClient::sendRawString(std::string &req)
{
    lock_guard<mutex> lock(mReqStrListMutex);
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
    ROS_DEBUG("Debug: init UDSClient thread");

    int ret;
    SESSION_t sess;
    list<string> reqList;

    sess.soc = mUdsSoc;
    sess.events = EPOLLIN;

    struct epoll_event event;
    struct epoll_event events[MAX_CLIENT_COUNT];

    event.events = sess.events;
    event.data.fd = sess.soc;
    if (epoll_ctl(mEpollfd, EPOLL_CTL_ADD, mUdsSoc, &event))
    {
        ROS_ERROR("Err: Failed to add DUS fd to epoll. Error[%d]: %s\n",
                  errno, strerror(errno));
        return;
    }

    ROS_DEBUG("Debug: UDSClient thread start\n");
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
                ROS_ERROR("Err: epoll fail. Error[%d]: %s\n", errno, strerror(errno));

                // break thread routine loop
                break;
            }
        }
        else if (ret == 0)
        {
            // timeout
        }
        else
        {
            assert(events[0].data.fd == mUdsSoc);

            // client socket
            sess.events = events[0].events;

            if (!onSoc(sess))
            {
                ROS_DEBUG("Debug: Remove socket[%d]\n", mUdsSoc);

                // remove socket from epoll
                epoll_ctl(mEpollfd, EPOLL_CTL_DEL, mUdsSoc, NULL);

                // break thread routine loop
                break;
            }
        }

        // process request signalings
        if (!mReqStrList.empty() && mReqStrListMutex.try_lock())
        {
            // swap buffer
            reqList.swap(mReqStrList);
            mReqStrListMutex.unlock();

            for (auto &req : reqList)
            {
                if (!sendSignalingToBuffer(sess, req))
                {
                    ROS_ERROR("Err: send signaling to buffer fail, drop signalings.");
                    break;
                }

                setWrite(sess, true);
            }

            reqList.clear();
        }
    }

    ROS_DEBUG("Debug: UDSClient thread stop\n");
}

bool UDSClient::onSoc(SESSION_t &sess)
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
        ROS_DEBUG("Debug: socket[%d] has been closed\n", sess.soc);
        return false;
    }
    if (sess.events & EPOLLERR)
    {
        // socket has been closed
        ROS_ERROR("Err: socket[%d]\n", sess.soc);
        return false;
    }
}

bool UDSClient::onRead(SESSION_t &sess)
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
    sess.recvBufEnd += nRet;

    // get signaling from raw buffer
    Document doc;
    if (!parseRawBuffer(sess, doc))
    {
        ROS_ERROR("Err: socket[%d] get signaling fail\n", sess.soc);
        return false;
    }

    // process signaling
    if (!mCb_OnSig(sess, doc))
    {
        ROS_ERROR("Err: socket[%d] process signaling fail\n", sess.soc);
        return false;
    }

    // adjust recv buffer pos variables
    if (sess.recvBufEnd == RECV_BUFFER_CAPACITY)
    {
        if (sess.recvBufPos == 0)
        {
            ROS_ERROR("Err: recv buffer of socket[%d] full\n", sess.soc);
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
        return setWrite(sess, true);
    }
}

bool UDSClient::onWrite(SESSION_t &sess)
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

        ROS_ERROR("Err: socket[%d] write fail. Error[%d]: %s\n", sess.soc, errno, strerror(errno));
        return false;
    }

    sess.sendBufPos += nRet;
    if (sess.recvBufPos == sess.recvBufEnd)
    {
        // no more data to send
        sess.recvBufPos = sess.recvBufEnd = 0;

        // remove epoll write event
        return setWrite(sess, false);
    }
}

bool UDSClient::parseRawBuffer(SESSION_t &sess, rapidjson::Document &sig)
{
    char *rawBuf = sess.recvBuf + sess.recvBufPos;
    int len = sess.recvBufEnd - sess.recvBufPos;

    int nRet = RCMP::parse(rawBuf, len, sig);
    if (nRet != SUCCESS)
    {
        if (nRet == NEED_MORE_DATA)
        {
            // need more data
            return true;
        }
        else
        {
            ROS_DEBUG("Debug: parse signaling fail: %d\n", nRet);
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

bool UDSClient::sendSignalingToBuffer(SESSION_t &sess, rapidjson::Document &sig)
{
    // get corresponding json string
    string jsonSig = RCMP::getJson(sig);

    return sendSignalingToBuffer(sess, jsonSig);
}

bool UDSClient::sendSignalingToBuffer(SESSION_t &sess, string &sig)
{
    // check empty send buffer size
    auto len = sig.length();
    if (RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd < len)
    {
        if (sess.sendBufPos == 0 ||
            (RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd + sess.sendBufPos < len))
        {
            // send buffer full
            ROS_DEBUG("Debug: send buffer fulll(soc: [%d]", sess.soc);
            return false;
        }

        // defrag buffer
        int bufLen = sess.sendBufEnd - sess.sendBufPos;
        memmove(sess.sendBuf, sess.sendBuf + sess.sendBufPos, bufLen);
        sess.sendBufPos = 0;
        sess.sendBufEnd = bufLen;
    }

    memcpy(sess.sendBuf + sess.sendBufEnd, sig.c_str(), len);
    sess.sendBufEnd += len;

    return true;
}

bool UDSClient::setWrite(SESSION_t &sess, bool write)
{
    // are there data in send buffer?
    if (write)
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
                ROS_ERROR("Err-UdsClient::setWrite: Failed to add poll-out event for client[%d]. Error[%d]: %s\n",
                          sess.soc, errno, strerror(errno));
                return false;
            }
        }
    }
    else
    {
        // set epoll write event for this socket
        if (sess.events & EPOLLOUT)
        {
            sess.events &= ~EPOLLOUT;

            struct epoll_event event;
            event.data.fd = sess.soc;
            event.events = sess.events;
            if (epoll_ctl(mEpollfd, EPOLL_CTL_MOD, sess.soc, &event))
            {
                ROS_ERROR("Err-UdsClient::setWrite: Failed to remove poll-out event for client[%d]. Error[%d]: %s\n",
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
