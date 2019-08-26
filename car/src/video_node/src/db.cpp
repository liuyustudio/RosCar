#include "db.h"

#include <stdio.h>
#include "ros/ros.h"

#include "roscar_common/error.h"

using namespace std;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace videonode
{

DB::ReturnType DB::addVideo(const char *nodeId, const char *nodeVideoId, const char *videoInfo)
{
    ROS_DEBUG("[DB::addVideo] add video(%s.%s): %s",
              nodeId,
              nodeVideoId,
              videoInfo);

    if (hasVideo(nodeId, nodeVideoId))
    {
        char buf[MAX_ERRBUF_SIZE];
        snprintf(buf,
                 MAX_ERRBUF_SIZE,
                 "[DB::addVideo] video(%s.%s) existed.",
                 nodeId,
                 nodeVideoId);
        ROS_ERROR_STREAM(buf);

        return ReturnType(ERROR, string(buf));
    }

    Video_t video(nodeId, nodeVideoId, videoInfo, ++mVideoId);
    mVideoList.emplace_back(video);

    return ReturnType(SUCCESS, "");
}

DB::ReturnType DB::removeVideo(const char *nodeId, const char *nodeVideoId)
{
    ROS_DEBUG("[DB::removeVideo] remove video(%s.%s).",
              nodeId,
              nodeVideoId);

    for (auto it = mVideoList.begin(); it != mVideoList.end(); ++it)
    {
        if (0 == it->nodeId.compare(nodeId) &&
            0 == it->nodeVideoId.compare(nodeVideoId))
        {
            mVideoList.erase(it);
            return ReturnType(SUCCESS, "");
        }
    }

    char buf[MAX_ERRBUF_SIZE];
    snprintf(buf,
             MAX_ERRBUF_SIZE,
             "[DB::removeVideo] video(%s.%s) not exist.",
             nodeId,
             nodeVideoId);
    ROS_ERROR_STREAM(buf);

    return ReturnType(ERROR, string(buf));
}

bool DB::hasVideo(const char *nodeId, const char *nodeVideoId) const
{
    for (auto &video : mVideoList)
    {
        if (0 == video.nodeId.compare(nodeId) &&
            0 == video.nodeVideoId.compare(nodeVideoId))
        {
            return true;
        }
    }

    return false;
}

DB::ReturnType DB::getVideo(const char *nodeId, const char *nodeVideoId, Video_t &video) const
{
    ROS_DEBUG("[DB::getVideo] get video(%s.%s).",
              nodeId,
              nodeVideoId);

    for (auto it = mVideoList.begin(); it != mVideoList.end(); ++it)
    {
        if (0 == it->nodeId.compare(nodeId) &&
            0 == it->nodeVideoId.compare(nodeVideoId))
        {
            video = *it;
            return ReturnType(SUCCESS, "");
        }
    }

    char buf[MAX_ERRBUF_SIZE];
    snprintf(buf,
             MAX_ERRBUF_SIZE,
             "[DB::getVideo] video(%s.%s) not exist.",
             nodeId,
             nodeVideoId);
    ROS_ERROR_STREAM(buf);

    return ReturnType(ERROR, string(buf));
}

void DB::getVideoList(list<Video_t> &videoList) const
{
    ROS_DEBUG("[DB::getVideoList] get video list.");

    videoList.clear();
    for (auto it = mVideoList.begin(); it != mVideoList.end(); ++it)
    {
        videoList.push_back(*it);
    }
}

DB::ReturnType DB::findVideo(const uint32_t id, Video_t &video)
{
    for (auto &_video : mVideoList)
    {
        if (id == _video.id)
        {
            video = _video;
            return ReturnType(SUCCESS, "");
        }
    }

    char buf[MAX_ERRBUF_SIZE];
    snprintf(buf, MAX_ERRBUF_SIZE, "[DB::findVideo] video[%u] not exist.", id);
    ROS_ERROR_STREAM(buf);

    return ReturnType(ERROR, string(buf));
}

} // namespace videonode
} // namespace car
} // namespace roscar
