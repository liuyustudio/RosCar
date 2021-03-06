#include "ros/ros.h"
#include "roscar_common/error.h"
#include "pilot/Info.h"
#include "pilot/Move.h"
#include "camMgr.h"

bool onSrvInfo(pilot::Info::Request &req,
               pilot::Info::Response &res)
{
  ROS_DEBUG("query dev info");

  res.id = "string of id";
  res.type = "string of type";
  res.name = "string of name";

  return true;
}

bool onSrvMove(pilot::Move::Request &req,
               pilot::Move::Response &res)
{
  ROS_DEBUG("move [angle: %.2f, power: %.2f, duration: %.2f]",
            req.angle,
            req.power,
            req.duration);

  res.err_code = roscar::car::roscar_common::SUCCESS;
  res.err_msg = "";

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pilot");

  ros::NodeHandle n;

  ros::ServiceServer srv_info = n.advertiseService("info", onSrvInfo);
  ros::ServiceServer srv_move = n.advertiseService("move", onSrvMove);

  // CamMgr
  ROS_DEBUG("init CamMgr");
  roscar::car::pilot::CamMgr camMgr;
  if (!camMgr.start())
  {
    ROS_ERROR("start CamMgr fail.");
    return 1;
  }

  ROS_INFO("Pilot Ready.");
  ros::spin();

  return 0;
}
