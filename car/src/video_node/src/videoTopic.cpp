#include "videoTopic.h"

#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <sys/socket.h>
#include <netdb.h>
#include <resolv.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <exception>
#include "ros/ros.h"

#include "roscar_common/error.h"
#include "video_node/VideoStream.h"

using namespace std;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace videonode
{

VideoTopic::Session_t *VideoTopic::createSession(ros::NodeHandle &nh,
                                                 int epollfd,
                                                 const char *host,
                                                 int port)
{
    // create session
    Session_t *pSession = new Session_t;
    if (!pSession)
    {
        ROS_ERROR("[VideoTopic::createSession] create session fail");
        return nullptr;
    }

    try
    {
        // init session
        if (initSession(pSession, nh, epollfd, host, port))
        {
            return pSession;
        }

        ROS_ERROR("[VideoTopic::createSession] init session fail");
    }
    catch (const exception &e)
    {
        ROS_ERROR_STREAM("[VideoTopic::createSession] catch an exception."
                         << e.what());
    }

    // close session and delete it
    releaseSession(pSession);
    delete pSession;

    return nullptr;
}

void VideoTopic::releaseSession(VideoTopic::Session_t *pSession)
{
    // release socket
    if (pSession->fd)
    {
        close(pSession->fd);
        pSession->fd = 0;
        pSession->events = 0;
    }
    // release ROS topic publisher
    if (pSession->publisher)
    {
        pSession->publisher.shutdown();
    }
}

bool VideoTopic::onRead(VideoTopic::Session_t *pSession)
{
    char buffer[STREAM_ERRBUF_SIZE];

    int nRet = recv(pSession->fd, buffer, STREAM_ERRBUF_SIZE, 0);
    if (nRet < 0)
    {
        ROS_ERROR("[VideoTopic::onRead] session[%d] read fail. Error[%d]: %s",
                  pSession->fd, errno, strerror(errno));
        return false;
    }
    else if (nRet == 0)
    {
        ROS_DEBUG("[VideoTopic::onRead] session[%d] closed by peer", pSession->fd);
        return false;
    }

    // send to corresponding topic
    return relay(pSession, buffer, nRet);
}

bool VideoTopic::relay(VideoTopic::Session_t *pSession, const void *buffer, int bufLen)
{
    // relay streaming data only if there are audiences.
    if (0 < pSession->publisher.getNumSubscribers())
    {
        video_node::VideoStream msg;

        msg.buffer.resize(bufLen);
        memcpy(msg.buffer.data(), buffer, bufLen);

        pSession->publisher.publish(msg);
    }

    return true;
}

bool VideoTopic::initSession(Session_t *pSession, ros::NodeHandle &nh,
                             int epollfd, const char *host, int port)
{
    pSession->host = host;
    pSession->port = port;

    // init socket (connect to service on video host)
    {
        // create socket
        pSession->fd =
            socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (pSession->fd < 0)
        {
            ROS_ERROR("[VideoTopic::initSession] create socket fail: %d-%s",
                      errno, strerror(errno));
            pSession->fd = 0;
            return false;
        }

        // init sockaddr_in struct
        struct sockaddr_in dest;
        bzero(&dest, sizeof(dest));
        dest.sin_family = AF_INET;
        dest.sin_port = htons(port);
        // TODO: gethostbyname
        if (inet_pton(AF_INET, host, &dest.sin_addr.s_addr) == 0)
        {
            ROS_ERROR("[VideoTopic::initSession] init sockaddr_in struct"
                      " for host[%s:%d] fail. Error[%d]: %s",
                      host, port, errno, strerror(errno));
            return false;
        }

        // Connect to host
        if (connect(pSession->fd, (struct sockaddr *)&dest, sizeof(dest)) != 0 &&
            EINPROGRESS != errno)
        {
            ROS_ERROR("[VideoTopic::initSession] Connect to host[%s:%d] fail. Error[%d]: %s",
                      host, port, errno, strerror(errno));
            return false;
        }
    }

    // create publisher
    {
        // generate random topic name
        assert(16 < TOPIC_NAME_LENGTH);
        static const char *BASE_TABLE = "0123456789ABCDEF";
        char topicName[TOPIC_NAME_LENGTH];
        char *p = topicName;

        srand(time(nullptr));
        for (int i = 0; i < 2; ++i)
        {
            int r = rand();
            for (int j = 0; j < 8; ++j, ++p, r >>= 4)
            {
                *p = BASE_TABLE[r & 0x0F];
            }
        }
        topicName[16] = 0;

        // create ROS publisher
        ROS_DEBUG("[VideoTopic::initSession] create ROS publisher[%s]", topicName);
        pSession->publisher =
            nh.advertise<video_node::VideoStream>(topicName,
                                                  ROS_PUBLISHER_QUEUE_SIZE);
        if (!pSession->publisher)
        {
            ROS_ERROR("[VideoTopic::initSession] create ROS publisher[%s] fail", topicName);
            return false;
        }
    }

    return true;
}

} // namespace videonode
} // namespace car
} // namespace roscar
