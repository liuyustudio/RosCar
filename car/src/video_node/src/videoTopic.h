#ifndef _ROSCAR_CAR_VIDEONODE_VIDEOTOPIC_H_
#define _ROSCAR_CAR_VIDEONODE_VIDEOTOPIC_H_

#include <sys/epoll.h>

#include "roscar_common/sessionBuffer.hpp"
#include "define.h"

namespace roscar
{
namespace car
{
namespace videonode
{

class VideoTopic
{
public:
    static const int TOPIC_NAME_LENGTH = 16;
    static const int STREAM_ERRBUF_SIZE = 4096;

    using SessionBufType = roscar_common::SessionBuffer<STREAM_ERRBUF_SIZE>;

    typedef struct _Session
    {
        char name[TOPIC_NAME_LENGTH + 4];
        int sourceFd;
        epoll_event event;
        SessionBufType buffer;
    } Session_t;

private:
    VideoTopic() = default;

public:
    static void initSession(Session_t &session);
};

} // namespace videonode
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_VIDEONODE_VIDEOTOPIC_H_
