#ifndef _ROSCAR_CAR_INTERFACE_TOPICMGR_H_
#define _ROSCAR_CAR_INTERFACE_TOPICMGR_H_

#include <map>
#include <string>
#include <thread>
#include <mutex>
#include "ros/ros.h"
#include "videoTopic.h"

namespace roscar
{
namespace car
{
namespace videonode
{

class TopicMgr
{
public:
    static const int INTERVAL_EPOLL_RETRY;
    static const int INTERVAL_CONNECT_RETRY;

    static const int EPOLL_MAX_EVENTS = 32;

    typedef struct _VideoStreamInfo
    {
        std::string host;
        int port;

        _VideoStreamInfo(const char *_host, int _port)
        {
            host = _host;
            port = _port;
        }
        _VideoStreamInfo(VideoTopic::Session_t *pSession)
            : _VideoStreamInfo(pSession->host.c_str(), pSession->port) {}
        const _VideoStreamInfo &operator=(const _VideoStreamInfo &src)
        {
            host = src.host;
            port = src.port;
        }
    } VideoStreamInfo_t;

    typedef struct _Comparator
    {
        bool operator()(const VideoStreamInfo_t &a,
                        const VideoStreamInfo_t &b) const
        {
            return std::make_pair(a.host, a.port) <
                   std::make_pair(b.host, b.port);
        }
    } Comparator_t;

    TopicMgr() : mStopFlag(false) {}
    virtual ~TopicMgr() = default;

    bool start();
    void stop();
    void join();

    bool createSession(ros::NodeHandle &nh, const char *host, int port);
    void closeSession(const char *host, int port);

protected:
    void threadFunc(TopicMgr *pTopicMgr);
    bool initEnv();
    void closeEnv();

    bool onSoc(unsigned int socEvents, VideoTopic::Session_t *pSession);

    int mEpollfd;

    std::vector<std::thread> mThreads;
    bool mStopFlag;

    std::mutex mAccessMutex;
    std::map<VideoStreamInfo_t,
             VideoTopic::Session_t *,
             Comparator_t>
        mSessionMap;
};

} // namespace videonode
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_TOPICMGR_H_
