#include "ros/ros.h"
#include "pilot/Info.h"

bool info(pilot::Info::Request  &req,
          pilot::Info::Response &res)
{
  res.jsonInfo = ""
    "{"
      "\"id\":\"string of id\","
      "\"type\":\"string of type\","
      "\"name\":\"string of name\""
    "}";
  ROS_INFO("request info");
  ROS_INFO("sending back response: [%s]", res.jsonInfo.c_str());
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
