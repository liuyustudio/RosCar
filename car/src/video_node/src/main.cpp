#include "ros/ros.h"
#include "roscar_common/error.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "video_node");

  ROS_INFO("Video Node Ready.");
  ros::spin();

  return 0;
}
