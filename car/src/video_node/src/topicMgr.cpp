#include "topicMgr.h"

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
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace videonode
{

const int TopicMgr::INTERVAL_EPOLL_RETRY = 100;
const int TopicMgr::INTERVAL_CONNECT_RETRY = 7;

bool TopicMgr::start()
{
    // start thread
    {
        if (!mStopFlag)
        {
            ROS_ERROR("[TopicMgr::start] stop thread first");
            return false;
        }

        ROS_DEBUG("[TopicMgr::start] start thread");
        mStopFlag = false;
        mThreads.emplace_back(&TopicMgr::threadFunc, this);
    }

    return true;
}

void TopicMgr::stop()
{
    // stop threads
    {
        ROS_DEBUG("[TopicMgr::stop] stop threads");
        if (!mStopFlag)
        {
            mStopFlag = true;
            for (auto &t : mThreads)
            {
                t.join();
            }
            mThreads.clear();
        }
        else
        {
            ROS_DEBUG("[TopicMgr::stop] threads not running");
        }
    }
}

void TopicMgr::join()
{
    for (auto &t : mThreads)
        t.join();
}

void TopicMgr::threadFunc()
{
    ROS_DEBUG("[TopicMgr::threadFunc] TopicMgr thread start");

    while (!mStopFlag)
    {
        // init env
        if (!initEnv())
        {
            ROS_ERROR("[TopicMgr::threadFunc] init fail. sleep %d seconds",
                      INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
            continue;
        }

        // main routine
        try
        {
            map<int, VideoTopic::Session_t> soc2Session;
            struct epoll_event events[EPOLL_MAX_EVENTS];

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
                        ROS_ERROR("[TopicMgr::threadFunc] epoll fail. Error[%d]: %s",
                                  errno, strerror(errno));
                        break;
                    }
                }
                else if (nRet == 0)
                {
                    // timeout
                    continue;
                }

                for (int i = 0; i < nRet; ++i)
                {
                    int soc = events[i].data.fd;
                    if (!onSoc(events[i].events, soc2Session[soc]))
                    {
                        ROS_DEBUG("[TopicMgr::threadFunc] Remove soc[%d]", soc);

                        // unbind video stream socket and session
                        soc2Session.erase(soc);
                        // remove socket from epoll
                        epoll_ctl(mEpollfd, EPOLL_CTL_DEL, soc, NULL);
                    }
                }
            }
        }
        catch (const exception &e)
        {
            ROS_ERROR_STREAM("[TopicMgr::threadFunc] catch an exception." << e.what());
        }

        // close env
        closeEnv();

        if (!mStopFlag)
        {
            ROS_DEBUG("[TopicMgr::threadFunc] sleep %d secnds and try again", INTERVAL_CONNECT_RETRY);
            this_thread::sleep_for(chrono::seconds(INTERVAL_CONNECT_RETRY));
        }
    }

    ROS_DEBUG("[TopicMgr::threadFunc] TopicMgr thread stop");
}

bool TopicMgr::initEnv()
{
    // create epoll
    ROS_DEBUG("[TopicMgr::initEnv] create epoll");
    if ((mEpollfd = epoll_create1(0)) < 0)
    {
        ROS_ERROR("[TopicMgr::initEnv] Failed to create epoll. Error[%d]: %s",
                  errno, strerror(errno));
        closeEnv();
        return false;
    }

    return true;
}

void TopicMgr::closeEnv()
{
    // close epoll file descriptor
    ROS_DEBUG("[TopicMgr::closeEnv] close epoll file descriptor");
    if (mEpollfd)
    {
        if (close(mEpollfd))
        {
            ROS_ERROR("[TopicMgr::closeEnv] Fail to close epoll. Error[%d]: %s",
                      errno, strerror(errno));
        }
        mEpollfd = 0;
    }
}

bool TopicMgr::onSoc(unsigned int socEvents, VideoTopic::Session_t &session)
{
    if (socEvents & EPOLLIN)
    {
        // available for read
        if (!onRead(session))
        {
            return false;
        }
    }
    if (socEvents & (EPOLLRDHUP | EPOLLHUP))
    {
        // socket has been closed
        ROS_DEBUG("[TopicMgr::onSoc] soc[%d] has been closed", session.soc);
        return false;
    }
    if (socEvents & EPOLLERR)
    {
        // socket has been closed
        ROS_ERROR("[TopicMgr::onSoc] soc[%d] fail", session.soc);
        return false;
    }

    return true;
}

// bool TopicMgr::onRead(VideoTopic::Session_t &session)
// {
//     assert(session.validate());

//     // receive data from socket
//     void *p;
//     int len;

//     std::tie(p, len) = session.buffer.getRecvBuf();

//     int nRet = recv(session.soc, p, len, 0);
//     if (nRet < 0)
//     {
//         ROS_ERROR("[TopicMgr::onRead] client[%d] read fail. Error[%d]: %s",
//                   session.soc, errno, strerror(errno));
//         return false;
//     }
//     else if (nRet == 0)
//     {
//         ROS_DEBUG("[TopicMgr::onRead] client[%d] session closed by peer", session.soc);
//         return false;
//     }

//     // adjust end pos
//     session.buffer.incRecvEnd(nRet);

//     // parse signaling from raw buffer
//     Document doc;
//     while (!session.buffer.recvBufEmpty())
//     {
//         nRet = parseSig(session.buffer, doc);
//         if (nRet != SUCCESS)
//         {
//             if (nRet == NEED_MORE_DATA)
//             {
//                 // need more data
//                 break;
//             }
//             else
//             {
//                 ROS_ERROR("[TopicMgr::onRead] soc[%d] parse signaling fail", session.soc);
//                 return false;
//             }
//         }

//         // process signaling
//         if (!mOnSigCallbak(session.buffer, doc))
//         {
//             ROS_ERROR("[TopicMgr::onRead] soc[%d] process signaling fail", session.soc);
//             return false;
//         }
//     }

//     // is recv buffer full
//     if (session.buffer.recvBufFull())
//     {
//         ROS_ERROR("[TopicMgr::onRead] recv buf of soc[%d] full", session.soc);
//         return false;
//     }

//     // are there data in send buffer?
//     if (!session.buffer.sendBufEmpty())
//     {
//         return setSocWritable(session, true);
//     }

//     return true;
// }

// bool TopicMgr::onWrite(VideoTopic::Session_t &session)
// {
//     assert(session.validate());

//     void *p;
//     int len;
//     std::tie(p, len) = session.buffer.getSendData();

//     int nRet = send(session.soc, p, len, 0);
//     if (nRet < 0)
//     {
//         if (errno == EAGAIN || errno == EWOULDBLOCK)
//         {
//             return true;
//         }

//         ROS_ERROR("[TopicMgr::onWrite] soc[%d] write fail. Error[%d]: %s", session.soc, errno, strerror(errno));
//         return false;
//     }

//     // ROS_DEBUG("[TopicMgr::onWrite] soc[%d] send %d bytes", session.soc, nRet);

//     session.buffer.incSendStart(nRet);
//     if (session.buffer.sendBufEmpty())
//     {
//         // stop send

//         // ROS_DEBUG("[TopicMgr::onWrite] soc[%d] stop send", session.soc);

//         return setSocWritable(session, false);
//     }

//     return true;
// }

// bool TopicMgr::setSocWritable(VideoTopic::Session_t &session, bool writable)
// {
//     bool update = false;
//     struct epoll_event event;

//     if (writable)
//     {
//         // set EPOLLOUT
//         update = 0 == (session.events & EPOLLOUT) ? session.events |= EPOLLOUT, true : false;
//     }
//     else
//     {
//         // remove EPOLLOUT
//         update = 1 == (session.events & EPOLLOUT) ? session.events &= ~EPOLLOUT, true : false;
//     }

//     if (update)
//     {
//         event.data.fd = session.soc;
//         event.events = session.events;
//         if (epoll_ctl(mEpollfd, EPOLL_CTL_MOD, session.soc, &event))
//         {
//             ROS_ERROR("[TopicMgr::setSocWritable] epoll_ctl for soc[%d] fail. Error[%d]: %s",
//                       session.soc, errno, strerror(errno));
//             return false;
//         }
//     }

//     return true;
// }

// int TopicMgr::parseSig(BufType &buffer, Document &sig)
// {
//     char *p;
//     int len;
//     tie(p, len) = buffer.getRecvData();

//     int nRet = RCMP::parse(p, len, sig);
//     if (nRet != SUCCESS)
//     {
//         ROS_ERROR("[TopicMgr::parseSig] RCMP parse fail.");
//         return nRet;
//     }

//     // adjust buffer pos
//     buffer.incRecvStart(len);

//     return SUCCESS;
// }

} // namespace videonode
} // namespace car
} // namespace roscar
