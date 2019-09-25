#include "camMgr.h"
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdlib.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <exception>
#include <regex>
#include <ros/ros.h>
#include "roscar_common/error.h"
#include "roscar_common/utils.h"
#include "video_node/StreamRegister.h"
#include "video_node/StreamDeregister.h"

using namespace std;
using namespace ros;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace pilot
{

const char *CamMgr::DEFAULT_NODE_ID = "pilot_node";
const char *CamMgr::DEFAULT_VIDEO_ID = "default_video";

bool CamMgr::start()
{
    // get default ip address
    IPv4_t ip;
    if (!Utils::getDefaultIp(ip))
    {
        ROS_ERROR("[CamMgr::start] get default ip fail");
        return false;
    }
    ROS_DEBUG_STREAM("[CamMgr::start] default IP: " << ip);

    // open camera
    ROS_DEBUG("[CamMgr::start] open camera");
    Camera_t cam("defaultCam");

    int errCode;
    string errMsg;
    tie(errCode, errMsg) = openCamera(cam);
    if (errCode != SUCCESS)
    {
        ROS_ERROR("[CamMgr::start] oper camera fail: %d - %s",
                  errCode, errMsg.c_str());
        return false;
    }

    // register default video stream
    ROS_DEBUG("[CamMgr::start] register default video stream");
    ros::NodeHandle nh;
    ros::ServiceClient client;
    client = nh.serviceClient<video_node::StreamRegister>("register");
    video_node::StreamRegister srv;
    srv.request.addr = ip.str();
    srv.request.port = DEFAULT_STREAM_PORT;
    srv.request.node_id = DEFAULT_NODE_ID;
    srv.request.video_id = DEFAULT_VIDEO_ID;
    if (!client.call(srv))
    {
        ROS_INFO("[CamMgr::start] Register video stream fail: %d - %s",
                 srv.response.err_code, srv.response.err_msg.c_str());

        // close camera
        ROS_DEBUG("[CamMgr::start] close camera");
        Camera_t cam("defaultCam");
        closeCamera(cam);

        return false;
    }

    return true;
}

void CamMgr::stop()
{
    // deregister from video node
    ROS_DEBUG("[CamMgr::stop] deregister from video node");
    ros::NodeHandle nh;
    ros::ServiceClient client;
    client = nh.serviceClient<video_node::StreamDeregister>("deregister");

    video_node::StreamDeregister srv;
    srv.request.node_id = DEFAULT_NODE_ID;
    srv.request.video_id = DEFAULT_VIDEO_ID;
    if (!client.call(srv))
    {
        ROS_INFO("[CamMgr::start] deregister from video node fail: %d - %s",
                 srv.response.err_code, srv.response.err_msg.c_str());
    }

    // close camera
    ROS_DEBUG("[CamMgr::stop] close camera");
    Camera_t cam("defaultCam");
    closeCamera(cam);
}

bool CamMgr::isValid(string &uri)
{
    auto queryInfo = [&uri](int fd) -> bool {
        v4l2_capability cap;

        if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &cap))
        {
            ROS_ERROR("[CamMgr::isValid] Error in ioctl"
                      " VIDIOC_QUERYCAP of camera[%s] fail. %d: %s",
                      uri.c_str(), errno, strerror(errno));
            return false;
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        {
            ROS_ERROR("[CamMgr::isValid] camera[%s] is no video capture device",
                      uri.c_str());
            return false;
        }

        if (!(cap.capabilities & V4L2_CAP_READWRITE))
        {
            ROS_ERROR("[CamMgr::isValid] camera[%s] does not support read i/o",
                      uri.c_str());
            return false;
        }

        return true;
    };

    int camFd = 0;
    if ((camFd = open(uri.c_str(), O_RDONLY)), camFd < 0)
    {
        ROS_ERROR("[CamMgr::isValid] open camera[%s] fail. %d: %s",
                  uri.c_str(), errno, strerror(errno));
        return false;
    }

    bool result = queryInfo(camFd);
    close(camFd);
    return result;
}

void CamMgr::queryCams(vector<string> &uriArray)
{
    ROS_DEBUG("[CamMgr::queryCams] query camera device list");

    static const char *DEVROOTDIR = "/dev";
    static const char *VIDEODEVURI_REG = R"(/dev/video\d+)";

    struct dirent *entry;
    DIR *dir = opendir(DEVROOTDIR);

    if (dir == NULL)
    {
        ROS_ERROR("[CamMgr::queryCams] open directory[%s] fail. %d: %s",
                  DEVROOTDIR, errno, strerror(errno));
        return;
    }

    regex re(VIDEODEVURI_REG);
    smatch match;
    while ((entry = readdir(dir)) != NULL)
    {
        string uri(entry->d_name);
        try
        {
            if (regex_search(uri, match, re) && match.size() > 1 && isValid(uri))
            {
                ROS_DEBUG("[CamMgr::queryCams] found %s", uri.c_str());
                uriArray.push_back(uri);
            }
        }
        catch (regex_error &e)
        {
            ROS_ERROR_STREAM("[CamMgr::queryCams] catch exception: "
                             << e.what());
        }
    }
    closedir(dir);
}

ReturnType CamMgr::openCamera(Camera_t &camera)
{
    ROS_WARN("[CamMgr::openCamera] open default camera only.");

    static const char *CMD =
        "nohup raspivid -l -o tcp://0.0.0.0:5110 -t 0 -w 1280 -h 720 &";

    ROS_DEBUG_STREAM("[CamMgr::openCamera] open camera: " << CMD);
    system(CMD);

    return make_tuple(SUCCESS, "");
}

void CamMgr::closeCamera(Camera_t &camera)
{
    ROS_WARN("[CamMgr::openCamera] stop all camera(s).");

    static const char *CMD = "pkill -f raspivid";

    ROS_DEBUG_STREAM("[CamMgr::closeCamera] close camera: " << CMD);
    system(CMD);
}

} // namespace pilot
} // namespace car
} // namespace roscar
