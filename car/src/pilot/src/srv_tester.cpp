#include "ros/ros.h"
#include "pilot/Info.h"
#include "pilot/Move.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_srv_info");
  if (argc != 1)
  {
    ROS_INFO("usage: test_srv_info");
    return 1;
  }

  ros::NodeHandle n;

  // service: Info
  ros::ServiceClient sc_Info = n.serviceClient<pilot::Info>("info");
  pilot::Info srv_Info;
  if (sc_Info.call(srv_Info))
  {
    ROS_INFO("Info:");
    ROS_INFO("  id:   %s", srv_Info.response.id.c_str());
    ROS_INFO("  type: %s", srv_Info.response.type.c_str());
    ROS_INFO("  name: %s", srv_Info.response.id.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service Info");
    return 1;
  }

  // service: Move
  ros::ServiceClient sc_Move = n.serviceClient<pilot::Move>("info");
  pilot::Move srv_Move;

  srv_Move.request.angle = 12.5;
  srv_Move.request.power = 12.5;
  srv_Move.request.duration = 12.5;

  ROS_DEBUG("move request: [angle: %.2f, power: %.2f, duration: %.2f]",
            srv_Move.request.angle,
            srv_Move.request.power,
            srv_Move.request.duration);

  if (sc_Move.call(srv_Move))
  {
    ROS_INFO("move response: [errno: %d, errmsg: %s]",
             srv_Move.response.err_code,
             srv_Move.response.err_msg.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service Move");
    return 1;
  }

  return 0;
}
