#include "ros/ros.h"
#include "pilot/Info.h"

bool info(pilot::Info::Request &req,
          pilot::Info::Response &res)
{
  ROS_DEBUG("request device info");

  res.id = "string of id";
  res.type = "string of type";
  res.name = "string of name";

  ROS_DEBUG("sending info back");

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pilot_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("info", info);
  ROS_INFO("Pilot Node Ready.");
  ros::spin();

  return 0;
}
