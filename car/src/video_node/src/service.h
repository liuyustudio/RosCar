#ifndef _ROSCAR_CAR_VIDEONODE_SERVICE_H_
#define _ROSCAR_CAR_VIDEONODE_SERVICE_H_

#include <mutex>

#include "ros/ros.h"

#include "video_node/List.h"
#include "video_node/Register.h"
#include "video_node/Unregister.h"
#include "video_node/OpenTopic.h"
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

    static bool onList(::video_node::List::Request &req,
                       ::video_node::List::Response &res);
    static bool onRegister(::video_node::Register::Request &req,
                           ::video_node::Register::Response &res);
    static bool onUnregister(::video_node::Unregister::Request &req,
                             ::video_node::Unregister::Response &res);
    static bool onOpenTopic(::video_node::OpenTopic::Request &req,
                            ::video_node::OpenTopic::Response &res);

    static ros::NodeHandle gNodeHandle;
    static std::mutex gAccessMutex;

    static DB gVideoDB;
    static TopicMgr gTopicMgr;

    static ros::ServiceServer gSrvList;
    static ros::ServiceServer gSrvRegister;
    static ros::ServiceServer gSrvUnregister;
};

} // namespace videonode
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_VIDEONODE_SERVICE_H_
