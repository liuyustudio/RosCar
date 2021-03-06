#ifndef _ROSCAR_CAR_VIDEONODE_VIDEOTOPIC_H_
#define _ROSCAR_CAR_VIDEONODE_VIDEOTOPIC_H_

#include <time.h>
#include <sys/epoll.h>
#include <string>
#include <ros/ros.h>
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
    static const int TOPIC_NAME_LENGTH = 32;
    static const int STREAM_ERRBUF_SIZE = 1 << 20;
    static const int ROS_PUBLISHER_QUEUE_SIZE = 1 << 10;
    static const int TOPIC_MAX_IDEL_TIME = 15;

    using SessionBufType = roscar_common::SessionBuffer<STREAM_ERRBUF_SIZE>;

    typedef struct _Session
    {
        int fd;
        uint32_t events;
        std::string host;
        int port;
        ros::Publisher publisher;
        time_t lastPlayTime;

        _Session() : fd(0), events(0), lastPlayTime(0) {}
        _Session &operator=(const _Session &src)
        {
            fd = src.fd;
            events = src.events;
            host = src.host;
            port = src.port;
            publisher = src.publisher;
            lastPlayTime = src.lastPlayTime;
        }
    } Session_t;

private:
    VideoTopic() = default;

public:
    static Session_t *createSession(ros::NodeHandle &nh,
                                    int epollfd,
                                    const char *host,
                                    int port);
    static void releaseSession(Session_t *pSession);
    static bool onRead(time_t curTime, Session_t *pSession);
    static bool relay(time_t curTime,
                      Session_t *pSession,
                      const void *buffer,
                      int bufLen);

private:
    static bool initSession(Session_t *pSession, ros::NodeHandle &nh,
                            int epollfd, const char *host, int port);
};

} // namespace videonode
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_VIDEONODE_VIDEOTOPIC_H_
