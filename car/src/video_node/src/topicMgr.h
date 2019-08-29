#ifndef _ROSCAR_CAR_INTERFACE_TOPICMGR_H_
#define _ROSCAR_CAR_INTERFACE_TOPICMGR_H_

#include <map>
#include <string>
#include <thread>
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

    TopicMgr() : mStopFlag(false);
    virtual ~TopicMgr() = default;

    bool start();
    void stop();
    void join();

protected:
    void threadFunc();
    bool initEnv();
    void closeEnv();

    bool onSoc(unsigned int socEvents, VideoTopic::Session_t &sess);

    int mEpollfd;

    std::vector<std::thread> mThreads;
    bool mStopFlag;

    std::map<std::string, VideoTopic::Session_t> mSessionMap;
};

} // namespace videonode
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_TOPICMGR_H_
