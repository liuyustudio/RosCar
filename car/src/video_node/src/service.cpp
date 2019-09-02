#include "service.h"

#include <list>
#include <sstream>
#include <thread>

#include "roscar_common/error.h"
#include "define.h"

using namespace std;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace videonode
{

ros::NodeHandle Service::gNodeHandle;
std::mutex Service::gAccessMutex;
DB Service::gVideoDB;
TopicMgr Service::gTopicMgr;
ros::ServiceServer Service::gSrvList;
ros::ServiceServer Service::gSrvRegister;
ros::ServiceServer Service::gSrvUnregister;

bool Service::start(ros::NodeHandle &nodeHandle)
{
    // start topic manager
    ROS_DEBUG("start topic manager");
    if (!gTopicMgr.start())
    {
        ROS_ERROR("[Service::start] start topic manager fail");
    }

    // start services
    ROS_DEBUG("start services");
    gSrvList = gNodeHandle.advertiseService("list", onList);
    gSrvRegister = gNodeHandle.advertiseService("register", onRegister);
    gSrvUnregister = gNodeHandle.advertiseService("unregister", onUnregister);
    if (!gSrvList || !gSrvRegister || !gSrvUnregister)
    {
        ROS_ERROR("[Service::start] services start fail");
        stop();
        return false;
    }

    return true;
}

void Service::stop()
{
    // stop services
    ROS_DEBUG("[Service::stop] stop services");

    if (gSrvList)
    {
        gSrvList.shutdown();
    }
    if (gSrvRegister)
    {
        gSrvRegister.shutdown();
    }
    if (gSrvUnregister)
    {
        gSrvUnregister.shutdown();
    }

    // stop topic manager
    ROS_DEBUG("[Service::stop] stop topic manager");
    gTopicMgr.stop();
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
    ROS_DEBUG_STREAM("video_addr: " << req.addr);
    ROS_DEBUG_STREAM("video_port: " << req.port);

    lock_guard<mutex> lock(gAccessMutex);
    tie(res.err_code, res.err_msg) = gVideoDB.addVideo(req.node_id.c_str(),
                                                       req.video_id.c_str(),
                                                       req.addr.c_str(),
                                                       req.port);

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

bool Service::onOpenTopic(video_node::OpenTopic::Request &req,
                          video_node::OpenTopic::Response &res)
{
    ROS_DEBUG("start video topic:");
    ROS_DEBUG_STREAM("node_id: " << req.node_id);
    ROS_DEBUG_STREAM("video_id: " << req.video_id);

    const char *nodeId = req.node_id.c_str();
    const char *videoId = req.video_id.c_str();
    Video_t video;

    lock_guard<mutex> lock(gAccessMutex);
    tie(res.err_code, res.err_msg) =
        gVideoDB.getVideo(req.node_id.c_str(),
                          req.video_id.c_str(),
                          video);
    if (SUCCESS != res.err_code)
    {
        ROS_ERROR("[Service::onTopic] start video topic fail: %d:%s",
                  res.err_code, res.err_msg.c_str());
        return false;
    }

    if (!gTopicMgr.createSession(gNodeHandle, video.addr.c_str(), video.port))
    {
        ROS_ERROR("[Service::onTopic] create video topic session fail");
        return false;
    }

    return true;
}

} // namespace videonode
} // namespace car
} // namespace roscar
