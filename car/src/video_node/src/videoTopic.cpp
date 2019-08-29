#include "videoTopic.h"

#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <sys/socket.h>
#include <netdb.h>
#include <resolv.h>
#include <arpa/inet.h>
#include <unistd.h>
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

bool VideoTopic::initSession(Session_t &session, int epollfd, const char *host, int port)
{
    // init socket (to video stream host)
    {
        // create socket
        if ((session.fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            ROS_ERROR("[VideoTopic::initSession] create session socket fail. Error[%d]: %s",
                      errno, strerror(errno));
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
            ROS_ERROR("[VideoTopic::initSession] init sockaddr_in struct for host[%s:%d] fail. Error[%d]: %s",
                      host, port, errno, strerror(errno));
            close(session.fd);
            session.fd = 0;
            return false;
        }

        // Connect to host
        if (connect(session.fd, (struct sockaddr *)&dest, sizeof(dest)) != 0)
        {
            ROS_ERROR("[VideoTopic::initSession] Connect to host[%s:%d] fail. Error[%d]: %s",
                      host, port, errno, strerror(errno));
            close(session.fd);
            session.fd = 0;
            return false;
        }
    }

    // assign random topic name
    {
        static const char *BASE_TABLE = "0123456789ABCDEF";

        assert(16 < TOPIC_NAME_LENGTH);
        srand(time(nullptr));
        char buffer[16 + 1];
        char *p = buffer;
        for (int i = 0; i < 2; ++i)
        {
            int r = rand();
            for (int j = 0; j < 8; ++j, ++p, r >>= 4)
            {
                *p = BASE_TABLE[r & 0x0F];
            }
        }
        session.name[16] = 0;

        snprintf(session.name, TOPIC_NAME_LENGTH, "VT_%s", buffer);
    }

    return true;
}

void VideoTopic::closeSession(Session_t &session)
{
    session.name[0] = 0;
    session.events = 0;
    close(session.fd);
    session.fd = 0;
}

bool VideoTopic::onRead(Session_t &session)
{
    char buffer[STREAM_ERRBUF_SIZE];

    int nRet = recv(session.fd, buffer, STREAM_ERRBUF_SIZE, 0);
    if (nRet < 0)
    {
        ROS_ERROR("[VideoTopic::onRead] session[%d] read fail. Error[%d]: %s",
                  session.fd, errno, strerror(errno));
        return false;
    }
    else if (nRet == 0)
    {
        ROS_DEBUG("[VideoTopic::onRead] session[%d] closed by peer", session.fd);
        return false;
    }

    // send to corresponding topic
    return sendTopic(session);
}

bool VideoTopic::sendTopic(Session_t &session) {
    // TODO: send to corresponding topic
    ROS_ERROR("TODO: send to corresponding topic[%s]", session.name);

    return true;
}

} // namespace videonode
} // namespace car
} // namespace roscar
