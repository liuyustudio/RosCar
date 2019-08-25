#ifndef _ROSCAR_CAR_VIDEONODE_DEFINE_H_
#define _ROSCAR_CAR_VIDEONODE_DEFINE_H_

#include <string>
#include <sstream>

#include "ros/ros.h"

namespace roscar
{
namespace car
{
namespace videonode
{

typedef struct _Video
{
    std::string nodeId;
    std::string nodeVideoId;
    std::string videoInfo;

    uint32_t id;

    template <typename T>
    _Video(T &_nodeId,
           T &_nodeVideoId,
           T &_videoInfo,
           const uint32_t _id = 0)
        : nodeId(_nodeId),
          nodeVideoId(_nodeVideoId),
          videoInfo(_videoInfo),
          id(id)
    {
    }
    _Video(const _Video &src)
    {
        *this = src;
    }
    const _Video &operator=(const _Video &src)
    {
        nodeId = src.nodeId;
        nodeVideoId = src.nodeVideoId;
        videoInfo = src.videoInfo;
        id = src.id;

        return *this;
    }

    std::string toJson()
    {
        std::stringstream ss;
        ss << "{"
           << R"("nodeId":")" << nodeId << R"(",)"
           << R"("nodeVideoId":")" << nodeVideoId << R"(",)"
           << R"("videoInfo":")" << videoInfo << R"(",)"
           << R"("id":)" << id
           << "}";
        std::move(ss.str());
    }

} Video_t;

} // namespace videonode
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_VIDEONODE_DEFINE_H_
