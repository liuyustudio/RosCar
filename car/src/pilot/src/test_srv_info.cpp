#include "ros/ros.h"
#include "pilot/Info.h"
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_srv_info");
  if (argc != 1)
  {
    ROS_INFO("usage: test_srv_info");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pilot::Info>("info");
  pilot::Info srv;
  if (client.call(srv))
  {
    ROS_INFO("Info: %s", srv.response.jsonInfo.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service info");
    return 1;
  }

  return 0;
}
