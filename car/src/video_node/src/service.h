#ifndef _ROSCAR_CAR_VIDEONODE_SERVICE_H_
#define _ROSCAR_CAR_VIDEONODE_SERVICE_H_

#include <mutex>

#include "ros/ros.h"

#include "video_node/StreamList.h"
#include "video_node/StreamRegister.h"
#include "video_node/StreamDeregister.h"
#include "video_node/StreamForward.h"
#include "db.h"
#include "topicMgr.h"

namespace roscar
{
namespace car
{
namespace videonode
{

class Service
{
public:
    static bool start(ros::NodeHandle &nodeHandle);
    static void stop();

protected:
    Service() = default;
    virtual ~Service() = default;

    static bool onList(video_node::StreamList::Request &req,
                       video_node::StreamList::Response &res);
    static bool onRegister(video_node::StreamRegister::Request &req,
                           video_node::StreamRegister::Response &res);
    static bool onDeregister(video_node::StreamDeregister::Request &req,
                             video_node::StreamDeregister::Response &res);
    static bool onOpenTopic(video_node::StreamForward::Request &req,
                            video_node::StreamForward::Response &res);

    static Service *gpService;

    std::mutex mAccessMutex;

    DB mVideoDB;
    TopicMgr mTopicMgr;

    ros::ServiceServer mSrvList;
    ros::ServiceServer mSrvRegister;
    ros::ServiceServer mSrvDeregister;
};

} // namespace videonode
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_VIDEONODE_SERVICE_H_
