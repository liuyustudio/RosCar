#ifndef _ROSCAR_CAR_VIDEONODE_DB_H_
#define _ROSCAR_CAR_VIDEONODE_DB_H_

#include <list>
#include <tuple>

#include "define.h"

namespace roscar
{
namespace car
{
namespace videonode
{

class DB
{
public:
    static const uint32_t MAX_ERRBUF_SIZE = 1024;

    using ReturnType = std::tuple<int, std::string>;

    DB() = default;
    virtual ~DB() = default;

    ReturnType addVideo(const char *nodeId,
                        const char *nodeVideoId,
                        const char *addr,
                        const int port);
    ReturnType removeVideo(const char *nodeId, const char *nodeVideoId);
    bool hasVideo(const char *nodeId, const char *nodeVideoId) const;
    ReturnType getVideo(const char *nodeId,
                        const char *nodeVideoId,
                        Video_t &video) const;
    void getVideoList(std::list<Video_t> &videoList) const;
    ReturnType findVideo(const uint32_t id, Video_t &video);

protected:
    std::list<Video_t> mVideoList;
    uint32_t mVideoId;
};

} // namespace videonode
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_VIDEONODE_DB_H_
