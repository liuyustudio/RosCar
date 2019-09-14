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

Service *Service::gpService = nullptr;

bool Service::start(ros::NodeHandle &nodeHandle)
{
    if (gpService)
    {
        ROS_INFO("Service started already.");
        return true;
    }

    gpService = new Service();
    if (!gpService)
    {
        ROS_ERROR("create Service instance fail.");
        return false;
    }

    // start topic manager
    ROS_DEBUG("start topic manager");
    if (!gpService->mTopicMgr.start(nodeHandle))
    {
        ROS_ERROR("[Service::start] start topic manager fail");
        stop();
        return false;
    }

    // init services
    ROS_DEBUG("init services");
    gpService->mSrvList = nodeHandle.advertiseService("list", onList);
    gpService->mSrvRegister = nodeHandle.advertiseService("register", onRegister);
    gpService->mSrvDeregister = nodeHandle.advertiseService("deregister", onDeregister);
    if (!gpService->mSrvList ||
        !gpService->mSrvRegister ||
        !gpService->mSrvDeregister)
    {
        ROS_ERROR("[Service::start] services init fail");
        stop();
        return false;
    }

    return true;
}

void Service::stop()
{
    if (gpService)
    {
        // stop services
        ROS_DEBUG("[Service::stop] stop services");

        if (gpService->mSrvList)
        {
            ROS_DEBUG("[Service::stop] shutdown service object: gpService->mSrvList");
            gpService->mSrvList.shutdown();
        }
        if (gpService->mSrvRegister)
        {
            ROS_DEBUG("[Service::stop] shutdown service object: gpService->mSrvRegister");
            gpService->mSrvRegister.shutdown();
        }
        if (gpService->mSrvDeregister)
        {
            ROS_DEBUG("[Service::stop] shutdown service object: gpService->mSrvDeregister");
            gpService->mSrvDeregister.shutdown();
        }

        // stop topic manager
        ROS_DEBUG("[Service::stop] stop topic manager");
        gpService->mTopicMgr.stop();

        // release Service instance
        ROS_DEBUG("[Service::stop] release Service instance");
        delete gpService;
        gpService = nullptr;
    }
}

bool Service::onList(video_node::StreamList::Request &req,
                     video_node::StreamList::Response &res)
{
    ROS_DEBUG("list registed video source");

    // load video stream list from inner DB

    stringstream ss;
    bool first = true;

    ss << "[";
    // query video list
    {
        list<Video_t> videoList;

        lock_guard<mutex> lock(gpService->mAccessMutex);
        gpService->mVideoDB.getVideoList(videoList);
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

bool Service::onRegister(video_node::StreamRegister::Request &req,
                         video_node::StreamRegister::Response &res)
{
    ROS_DEBUG("video source registerStream:");
    ROS_DEBUG_STREAM("node_id: " << req.node_id);
    ROS_DEBUG_STREAM("video_id: " << req.video_id);
    ROS_DEBUG_STREAM("video_addr: " << req.addr);
    ROS_DEBUG_STREAM("video_port: " << req.port);

    lock_guard<mutex> lock(gpService->mAccessMutex);
    tie(res.err_code, res.err_msg) = gpService->mVideoDB.addVideo(req.node_id.c_str(),
                                                                  req.video_id.c_str(),
                                                                  req.addr.c_str(),
                                                                  req.port);

    return SUCCESS == res.err_code;
}

bool Service::onDeregister(video_node::StreamDeregister::Request &req,
                           video_node::StreamDeregister::Response &res)
{
    ROS_DEBUG("DeregisterStream video source:");
    ROS_DEBUG_STREAM("node_id: " << req.node_id);
    ROS_DEBUG_STREAM("video_id: " << req.video_id);

    lock_guard<mutex> lock(gpService->mAccessMutex);
    tie(res.err_code, res.err_msg) =
        gpService->mVideoDB.removeVideo(req.node_id.c_str(),
                                        req.video_id.c_str());

    return SUCCESS == res.err_code;
}

bool Service::onOpenTopic(video_node::StreamForward::Request &req,
                          video_node::StreamForward::Response &res)
{
    ROS_DEBUG("start video topic:");
    ROS_DEBUG_STREAM("node_id: " << req.node_id);
    ROS_DEBUG_STREAM("video_id: " << req.video_id);

    const char *nodeId = req.node_id.c_str();
    const char *videoId = req.video_id.c_str();
    Video_t video;

    lock_guard<mutex> lock(gpService->mAccessMutex);
    tie(res.err_code, res.err_msg) =
        gpService->mVideoDB.getVideo(req.node_id.c_str(),
                                     req.video_id.c_str(),
                                     video);
    if (SUCCESS != res.err_code)
    {
        ROS_ERROR("[Service::onTopic] start video topic fail: %d:%s",
                  res.err_code, res.err_msg.c_str());
        return false;
    }

    if (!gpService->mTopicMgr.createSession(video.addr.c_str(), video.port))
    {
        ROS_ERROR("[Service::onTopic] create video topic session fail");
        return false;
    }

    return true;
}

} // namespace videonode
} // namespace car
} // namespace roscar
