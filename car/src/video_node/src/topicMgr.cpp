#include "topicMgr.h"

#include <assert.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/epoll.h>
#include <chrono>
#include <exception>

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
    // init environment
    if (!initEnv())
    {
        ROS_ERROR("[TopicMgr::start] init environment fail");
        return false;
    }

    // start thread
    {
        if (!mStopFlag)
        {
            ROS_ERROR("[TopicMgr::start] stop thread first");
            return false;
        }

        ROS_DEBUG("[TopicMgr::start] start thread");
        mStopFlag = false;
        mThreads.emplace_back(&TopicMgr::threadFunc, this, this);
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

    // close environment
    closeEnv();
}

void TopicMgr::join()
{
    for (auto &t : mThreads)
        t.join();
}

void TopicMgr::threadFunc(TopicMgr *pTopicMgr)
{
    ROS_DEBUG("[TopicMgr::threadFunc] TopicMgr thread start");

    while (!mStopFlag)
    {
        // main routine
        try
        {
            struct epoll_event events[EPOLL_MAX_EVENTS];

            while (!mStopFlag)
            {
                int nRet = epoll_wait(mEpollfd,
                                      events,
                                      EPOLL_MAX_EVENTS,
                                      INTERVAL_EPOLL_RETRY);
                if (nRet < 0)
                {
                    if (errno == EAGAIN || errno == EINTR)
                    {
                        this_thread::sleep_for(
                            chrono::milliseconds(INTERVAL_EPOLL_RETRY));
                    }
                    else
                    {
                        ROS_ERROR("[TopicMgr::threadFunc] epoll fail: %d - %s",
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
                    VideoTopic::Session_t *pSession =
                        static_cast<VideoTopic::Session_t *>(events[i].data.ptr);
                    if (!onSoc(events[i].events, pSession))
                    {
                        ROS_DEBUG("[TopicMgr::threadFunc] Remove soc[%d]",
                                  pSession->fd);

                        // remove socket from epoll
                        epoll_ctl(mEpollfd, EPOLL_CTL_DEL, pSession->fd, NULL);

                        // release session object
                        VideoTopic::releaseSession(pSession);

                        // remove from session map
                        ROS_DEBUG("[TopicMgr::threadFunc] remove from session map");

                        lock_guard<mutex> lock(pTopicMgr->mAccessMutex);
                        VideoStreamInfo_t videoStreamInfo(pSession);
                        pTopicMgr->mSessionMap.erase(videoStreamInfo);
                        delete pSession;
                    }
                }
            }
        }
        catch (const exception &e)
        {
            ROS_ERROR_STREAM("[TopicMgr::threadFunc] catch an exception." << e.what());
        }

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

bool TopicMgr::onSoc(unsigned int socEvents, VideoTopic::Session_t *pSession)
{
    if (socEvents & EPOLLIN)
    {
        // available for read
        if (!VideoTopic::onRead(pSession))
        {
            return false;
        }
    }
    if (socEvents & (EPOLLRDHUP | EPOLLHUP))
    {
        // socket has been closed
        ROS_DEBUG("[TopicMgr::onSoc] soc[%d] has been closed", pSession->fd);
        return false;
    }
    if (socEvents & EPOLLERR)
    {
        // socket has been closed
        ROS_ERROR("[TopicMgr::onSoc] soc[%d] fail", pSession->fd);
        return false;
    }

    return true;
}

bool TopicMgr::createSession(ros::NodeHandle nh, const char *host, int port)
{
    lock_guard<mutex> lock(mAccessMutex);

    // searching in session map
    VideoStreamInfo_t videoStreamInfo(host, port);
    if (mSessionMap.find(videoStreamInfo) != mSessionMap.end())
    {
        // session existed
        ROS_ERROR("[TopicMgr::createSession] session for %s:%d existed", host, port);
        return false;
    }

    // create session object
    VideoTopic::Session_t *pSession =
        VideoTopic::createSession(nh, mEpollfd, host, port);
    if (!pSession)
    {
        ROS_ERROR("[TopicMgr::createSession] create session object fail");
        return false;
    }

    // add session into epoll
    pSession->events = EPOLLIN;
    struct epoll_event event;

    event.data.ptr = pSession;
    event.events = pSession->events;
    if (epoll_ctl(mEpollfd, EPOLL_CTL_ADD, pSession->fd, &event))
    {
        ROS_ERROR("[TopicMgr::createSession] epoll add fail. Error[%d]: %s",
                  errno, strerror(errno));
        VideoTopic::releaseSession(pSession);
        delete pSession;
    }

    mSessionMap[videoStreamInfo] = pSession;

    return true;
}

void TopicMgr::closeSession(const char *host, int port)
{
    VideoStreamInfo_t videoStreamInfo(host, port);

    lock_guard<mutex> lock(mAccessMutex);

    // searching in session map
    auto it = mSessionMap.find(videoStreamInfo);
    if (it == mSessionMap.end())
    {
        // session existed
        ROS_WARN("[TopicMgr::closeSession] session %s:%d not exist", host, port);
        return;
    }
    VideoTopic::Session_t *pSession = it->second;

    // remove session from epoll
    if (epoll_ctl(mEpollfd, EPOLL_CTL_DEL, pSession->fd, nullptr))
    {
        ROS_ERROR("[TopicMgr::closeSession] epoll remove fail. Error[%d]: %s",
                  errno, strerror(errno));
    }

    // remove from session map
    mSessionMap.erase(it);

    // release session
    // TODO: add reference count
    VideoTopic::releaseSession(pSession);
    delete pSession;
}

} // namespace videonode
} // namespace car
} // namespace roscar
