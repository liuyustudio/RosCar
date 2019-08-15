#include "udsClient.h"

#include <assert.h>
#include <errno.h>
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
    ROS_DEBUG("[UDSClient::start] init epoll file descriptor");
    if ((mEpollfd = epoll_create1(0)) < 0)
    {
        ROS_ERROR("[UDSClient::start] Failed to create epoll. Error[%d]: %s",
                  errno, strerror(errno));
        return false;
    }

    // init Unix Domain Socket
    ROS_DEBUG("[UDSClient::start] init Unix Domain Socket: %s", udsUri);
    {
        // create socket
        if ((mUdsSoc = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
        {
            ROS_ERROR("[UDSClient::start] Fail to create socket. Error[%d]: %s",
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
            ROS_ERROR("[UDSClient::start] Fail to connect to UDS[%s]. Error[%d]: %s",
                      udsUri, errno, strerror(errno));
            close(mUdsSoc);
            mUdsSoc = 0;
            return false;
        }
    }

    // start threads
    ROS_DEBUG("[UDSClient::start] start threads");
    mThreadArray.emplace_back(&UDSClient::threadFunc, this);

    return true;
}

void UDSClient::stop()
{
    // stop threads
    ROS_DEBUG("[UDSClient::stop] stop threads");
    mStopUDS = true;
    for (auto &t : mThreadArray)
    {
        t.join();
    }

    // close epoll file descriptor
    ROS_DEBUG("[UDSClient::stop] close epoll file descriptor");
    if (mEpollfd)
    {
        if (close(mEpollfd))
        {
            ROS_ERROR("[UDSClient::stop] Fail to close epoll. Error[%d]: %s",
                      errno, strerror(errno));
        }
        mEpollfd = 0;
    }

    // close Unix Domain Socket
    ROS_DEBUG("[UDSClient::stop] close Unix Domain Socket");
    if (mUdsSoc)
    {
        if (close(mUdsSoc))
        {
            ROS_ERROR("[UDSClient::stop] Fail to close Unix Domain Socket. Error[%d]: %s",
                      errno, strerror(errno));
        }
        mUdsSoc = 0;
    }
}

bool UDSClient::sendRawString(std::string &req)
{
    ROS_DEBUG("[UDSClient::sendRawString] put sig into sending queue");
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
    ROS_DEBUG("[UDSClient::threadFunc] init UDSClient thread");

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
        ROS_ERROR("[UDSClient::threadFunc] Failed to add DUS fd to epoll. Error[%d]: %s",
                  errno, strerror(errno));
        return;
    }

    ROS_DEBUG("[UDSClient::threadFunc] UDSClient thread start");
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
                ROS_ERROR("[UDSClient::threadFunc] epoll fail. Error[%d]: %s", errno, strerror(errno));

                // break thread routine loop
                break;
            }
        }
        else if (ret == 0)
        {
            // epoll wait timeout
        }
        else
        {
            assert(events[0].data.fd == mUdsSoc);

            if (!onSoc(events[0].events, sess))
            {
                ROS_DEBUG("[UDSClient::threadFunc] Remove socket[%d]", mUdsSoc);

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
                if (!sendToBuf(sess, req))
                {
                    ROS_ERROR("[UDSClient::threadFunc] send signaling to buffer fail, drop signalings.");
                    break;
                }

                setWriteFlag(sess, true);
            }

            reqList.clear();
        }
    }

    ROS_DEBUG("[UDSClient::threadFunc] UDSClient thread stop");
}

bool UDSClient::onSoc(unsigned int socEvents, SESSION_t &sess)
{
    if (socEvents & EPOLLIN)
    {
        // available for read
        if (!onRead(sess))
        {
            ROS_DEBUG("[UDSClient::onSoc] read fail");
            return false;
        }
    }
    if (socEvents & EPOLLOUT)
    {
        // available for write
        if (!onWrite(sess))
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
        ROS_ERROR("[UDSClient::onRead] client[%d] read fail. Error[%d]: %s",
                  sess.soc, errno, strerror(errno));
        return false;
    }
    sess.recvBufEnd += nRet;

    // get signaling from raw buffer
    Document doc;
    nRet = parseRawBuffer(sess, doc);
    if (SUCCESS != nRet)
    {
        if (nRet == NEED_MORE_DATA)
        {
            // need more data
        }
        else
        {
            ROS_DEBUG("[UDSClient::onRead] parse signaling fail: %d", nRet);
            return false;
        }

        ROS_ERROR("[UDSClient::onRead] socket[%d] get signaling fail", sess.soc);
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
        return setWriteFlag(sess, true);
    }

    return true;
}

bool UDSClient::onWrite(SESSION_t &sess)
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

    sess.sendBufPos += nRet;
    if (sess.recvBufPos == sess.recvBufEnd)
    {
        // no more data to send
        sess.recvBufPos = sess.recvBufEnd = 0;

        // remove epoll write event
        return setWriteFlag(sess, false);
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

bool UDSClient::setWriteFlag(SESSION_t &sess, bool writeFlag)
{
    // are there data in send buffer?
    if (writeFlag)
    {
        // set epoll write flag for this socket
        if (!(sess.events & EPOLLOUT))
        {
            ROS_DEBUG("[UDSClient::setWriteFlag] set send flag");
            sess.events |= EPOLLOUT;

            struct epoll_event event;
            event.data.fd = sess.soc;
            event.events = sess.events;
            if (epoll_ctl(mEpollfd, EPOLL_CTL_MOD, sess.soc, &event))
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
            ROS_DEBUG("[UDSClient::setWriteFlag] remove send flag");
            sess.events &= ~EPOLLOUT;

            struct epoll_event event;
            event.data.fd = sess.soc;
            event.events = sess.events;
            if (epoll_ctl(mEpollfd, EPOLL_CTL_MOD, sess.soc, &event))
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
