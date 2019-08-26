#ifndef _ROSCAR_CAR_VIDEONODE_SERVICE_H_
#define _ROSCAR_CAR_VIDEONODE_SERVICE_H_

#include <mutex>

#include "ros/ros.h"

#include "video_node/List.h"
#include "video_node/Register.h"
#include "video_node/Unregister.h"
#include "db.h"

namespace roscar
{
namespace car
{
namespace videonode
{

class Service
{
public:
    Service(ros::NodeHandle &nodeHandle);
    virtual ~Service() = default;

protected:
    static bool onList(::video_node::List::Request &req,
                       ::video_node::List::Response &res);
    static bool onRegister(::video_node::Register::Request &req,
                           ::video_node::Register::Response &res);
    static bool onUnregister(::video_node::Unregister::Request &req,
                             ::video_node::Unregister::Response &res);

    static std::mutex gAccessMutex;
    static DB gVideoDB;

    ros::NodeHandle mNodeHandle;

    ros::ServiceServer mSrvList;
    ros::ServiceServer mSrvRegister;
    ros::ServiceServer mSrvUnregister;
};

} // namespace videonode
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_VIDEONODE_SERVICE_H_
