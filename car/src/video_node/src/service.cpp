#include "service.h"

#include <list>
#include <sstream>
#include <thread>

#include "roscar_common/error.h"

using namespace std;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace videonode
{

mutex Service::gAccessMutex;
DB Service::gVideoDB;

Service::Service(ros::NodeHandle &nodeHandle)
    : mNodeHandle(nodeHandle)
{
    // init services
    ROS_DEBUG("init services");
    mSrvList = mNodeHandle.advertiseService("list", onList);
    mSrvRegister = mNodeHandle.advertiseService("register", onRegister);
    mSrvUnregister = mNodeHandle.advertiseService("unregister", onUnregister);
}

bool Service::onList(video_node::List::Request &req,
                     video_node::List::Response &res)
{
    ROS_DEBUG("list registed video source");

    // load video stream list from inner DB

    stringstream ss;
    bool first = true;

    ss << "[";
    // query video list
    {
        list<Video_t> videoList;

        lock_guard<mutex> lock(gAccessMutex);
        gVideoDB.getVideoList(videoList);
        for (auto &video : videoList)
        {
            first ? first = false : (ss << ",", true);
            ss << video.toJson();
        }
    }
    ss << "]";

    res.video_list = ss.str();

    return true;
}

bool Service::onRegister(video_node::Register::Request &req,
                         video_node::Register::Response &res)
{
    ROS_DEBUG("video source register:");
    ROS_DEBUG_STREAM("node_id: " << req.node_id);
    ROS_DEBUG_STREAM("video_id: " << req.video_id);
    ROS_DEBUG_STREAM("video_info: " << req.video_info);

    lock_guard<mutex> lock(gAccessMutex);
    tie(res.err_code, res.err_msg) = gVideoDB.addVideo(req.node_id.c_str(),
                                                       req.video_id.c_str(),
                                                       req.video_info.c_str());

    return SUCCESS == res.err_code;
}

bool Service::onUnregister(video_node::Unregister::Request &req,
                           video_node::Unregister::Response &res)
{
    ROS_DEBUG("unregister video source:");
    ROS_DEBUG_STREAM("node_id: " << req.node_id);
    ROS_DEBUG_STREAM("video_id: " << req.video_id);

    lock_guard<mutex> lock(gAccessMutex);
    tie(res.err_code, res.err_msg) =
        gVideoDB.removeVideo(req.node_id.c_str(),
                             req.video_id.c_str());

    return SUCCESS == res.err_code;
}

} // namespace videonode
} // namespace car
} // namespace roscar
