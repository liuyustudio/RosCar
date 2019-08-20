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

UDS::UDS(OnSigCallbak onSigCallbak)
    : mUdsUri(),
      mEpollfd(0),
      mSrvSoc(0),
      mStopFlag(true),
      mOnSigCallbak(onSigCallbak)
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
    // stop threads
    {
        ROS_DEBUG("[UDS::stop] stop threads");
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
        // init env
        if (!initEnv())
        {
            ROS_ERROR("[UDS::threadFunc] init fail. sleep %d seconds",
                      INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
            continue;
        }

        // main routine
        try
        {
            map<int, UdsSession_t> soc2Session;

            struct epoll_event event;
            struct epoll_event events[EPOLL_MAX_EVENTS];

            // add server socket into epoll
            event.events = EPOLLIN;
            event.data.fd = mSrvSoc;
            if (epoll_ctl(mEpollfd, EPOLL_CTL_ADD, mSrvSoc, &event))
            {
                ROS_ERROR("[UDS::threadFunc] add server socket into epoll fail. Error[%d]: %s",
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
                        ROS_ERROR("[UDS::threadFunc] epoll fail. Error[%d]: %s",
                                  errno, strerror(errno));
                        break;
                    }
                }
                else if (nRet == 0)
                {
                    // timeout
                }
                else
                {
                    for (int i = 0; i < nRet; ++i)
                    {
                        if (events[i].data.fd == mSrvSoc)
                        {
                            // UDS server socket

                            // accept client
                            int soc = accept(mSrvSoc, NULL, NULL);
                            if (soc == -1)
                            {
                                ROS_ERROR("[UDS::threadFunc] accept fail. Error[%d]: %s", errno, strerror(errno));
                                continue;
                            }

                            // bind client socket with session data
                            soc2Session[soc] = UdsSession_t(soc, EPOLLIN);

                            // add client soc into epoll
                            event.data.fd = soc;
                            event.events = EPOLLIN;
                            if (epoll_ctl(mEpollfd, EPOLL_CTL_ADD, soc, &event))
                            {
                                ROS_ERROR("[UDS::threadFunc] Failed to add client fd to epoll. Error[%d]: %s",
                                          errno, strerror(errno));
                                soc2Session.erase(soc);
                                close(soc);
                                continue;
                            }

                            ROS_DEBUG("[UDS::threadFunc] Accept client[%d]", soc);
                        }
                        else
                        {
                            // client socket
                            int soc = events[i].data.fd;
                            UdsSession_t &session = soc2Session[soc];

                            if (!onSoc(events[i].events, session))
                            {
                                ROS_DEBUG("[UDS::threadFunc] Remove soc[%d]", soc);

                                // unbind client socket and session
                                soc2Session.erase(soc);
                                // remove socket from epoll
                                epoll_ctl(mEpollfd, EPOLL_CTL_DEL, soc, NULL);
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
        closeEnv();

        if (!mStopFlag)
        {
            ROS_DEBUG("[UDS::threadFunc] sleep %d secnds and try again", INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
        }
    }

    ROS_DEBUG("[UDS::threadFunc] UDS thread stop");
}

bool UDS::initEnv()
{
    // create epoll
    ROS_DEBUG("[UDS::initEnv] create epoll");
    if ((mEpollfd = epoll_create1(0)) < 0)
    {
        ROS_ERROR("[UDS::initEnv] Failed to create epoll. Error[%d]: %s",
                  errno, strerror(errno));
        closeEnv();
        return false;
    }

    // init Unix Domain Socket
    {
        ROS_DEBUG("[UDS::initEnv] init Unix Domain Socket: %s", mUdsUri.c_str());

        // create socket
        if ((mSrvSoc = socket(AF_UNIX, SOCK_STREAM, 0)) <= 0)
        {
            ROS_ERROR("[UDS::initEnv] Fail to create socket. Error[%d]: %s",
                      errno, strerror(errno));
            closeEnv();
            return false;
        }

        // bind
        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, mUdsUri.c_str(), sizeof(addr.sun_path) - 1);
        unlink(mUdsUri.c_str());
        if (bind(mSrvSoc, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        {
            ROS_ERROR("[UDS::initEnv] Fail to bind[%s]. Error[%d]: %s",
                      mUdsUri.c_str(), errno, strerror(errno));
            closeEnv();
            return false;
        }

        // listen
        if (listen(mSrvSoc, SOMAXCONN) == -1)
        {
            ROS_ERROR("[UDS::initEnv] Listen fail. Error[%d]: %s",
                      errno, strerror(errno));
            closeEnv();
            return false;
        }
    }

    return true;
}

void UDS::closeEnv()
{
    // close epoll file descriptor
    ROS_DEBUG("[UDS::closeEnv] close epoll file descriptor");
    if (mEpollfd)
    {
        if (close(mEpollfd))
        {
            ROS_ERROR("[UDS::closeEnv] Fail to close epoll. Error[%d]: %s",
                      errno, strerror(errno));
        }
        mEpollfd = 0;
    }

    // close Unix Domain Server Socket
    ROS_DEBUG("[UDS::closeEnv] close Unix Domain Server Socket");
    if (mSrvSoc)
    {
        if (close(mSrvSoc))
        {
            ROS_ERROR("[UDS::closeEnv] Close Unix Domain Server Socket fail. Error[%d]: %s",
                      errno, strerror(errno));
        }
        mSrvSoc = 0;
    }
}

bool UDS::onSoc(unsigned int socEvents, UdsSession_t &session)
{
    if (socEvents & EPOLLIN)
    {
        // available for read
        if (!onRead(session))
        {
            return false;
        }
    }
    if (socEvents & EPOLLOUT)
    {
        // available for write
        if (!onWrite(session))
        {
            return false;
        }
    }
    if (socEvents & (EPOLLRDHUP | EPOLLHUP))
    {
        // socket has been closed
        ROS_DEBUG("[UDS::onSoc] soc[%d] has been closed", session.soc);
        return false;
    }
    if (socEvents & EPOLLERR)
    {
        // socket has been closed
        ROS_ERROR("[UDS::onSoc] soc[%d]", session.soc);
        return false;
    }

    return true;
}

bool UDS::onRead(UdsSession_t &session)
{
    assert(session.validate());

    // receive data from socket
    void *p;
    int len;

    std::tie(p, len) = session.buffer.getRecvBuf();

    int nRet = recv(session.soc, p, len, 0);
    if (nRet < 0)
    {
        ROS_ERROR("[UDS::onRead] client[%d] read fail. Error[%d]: %s",
                  session.soc, errno, strerror(errno));
        return false;
    }
    else if (nRet == 0)
    {
        ROS_DEBUG("[UDS::onRead] client[%d] session closed by peer", session.soc);
        return false;
    }

    // adjust end pos
    session.buffer.incRecvEnd(nRet);

    // parse signaling from raw buffer
    Document doc;
    while (!session.buffer.recvBufEmpty())
    {
        nRet = parseSig(session.buffer, doc);
        if (nRet != SUCCESS)
        {
            if (nRet == NEED_MORE_DATA)
            {
                // need more data
                break;
            }
            else
            {
                ROS_ERROR("[UDS::onRead] soc[%d] parse signaling fail", session.soc);
                return false;
            }
        }

        // process signaling
        if (!mOnSigCallbak(session.buffer, doc))
        {
            ROS_ERROR("[UDS::onRead] soc[%d] process signaling fail", session.soc);
            return false;
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

bool UDS::onWrite(UdsSession_t &session)
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

        ROS_ERROR("[UDS::onWrite] soc[%d] write fail. Error[%d]: %s", session.soc, errno, strerror(errno));
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

bool UDS::setSocWritable(UdsSession_t &session, bool writable)
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
            ROS_ERROR("[UDS::setSocWritable] epoll_ctl for soc[%d] fail. Error[%d]: %s",
                      session.soc, errno, strerror(errno));
            return false;
        }
    }

    return true;
}

int UDS::parseSig(BufType &buffer, Document &sig)
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

} // namespace interface
} // namespace car
} // namespace roscar
