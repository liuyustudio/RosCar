#include "ros/ros.h"
#include "roscar_common/error.h"
#include "service.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_node");

    // init video node service
    ROS_DEBUG("init video node service");
    ros::NodeHandle nodeHandle;
    if (!roscar::car::videonode::Service::start(nodeHandle))
    {
        ROS_ERROR("init video node service fail.");
        return 1;
    }

    ROS_INFO("Video Node Ready.");
    ros::spin();

    return 0;
}
