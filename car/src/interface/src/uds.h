#ifndef _ROSCAR_CAR_INTERFACE_UDS_H_
#define _ROSCAR_CAR_INTERFACE_UDS_H_

#include "ros/ros.h"

namespace roscar
{
namespace car
{
namespace interface
{

class UDS
{
public:
    static const int EPOLL_TIMEOUT = 1 * 1000;
    static const int EPOLL_RETRY_INTERVAL = 1 * 100 * 1000;
    static const int EPOLL_MAX_EVENTS = 16;
    static const int MAX_CLIENT_COUNT = 8;
    static const int RECV_BUFFER_CAPACITY = 64 * 1024;
    static const int SEND_BUFFER_CAPACITY = 64 * 1024;
    static const char *UDS_PATH;

    typedef struct SESSION
    {
        int soc;
        unsigned int events;
        int recvBufPos = 0;
        int recvBufEnd = 0;
        int sendBufPos = 0;
        int sendBufEnd = 0;
        char recvBuf[RECV_BUFFER_CAPACITY];
        char sendBuf[SEND_BUFFER_CAPACITY];
    } SESSION_t;

    UDS();

    void threadFunc();
    bool onSession(SESSION_t &sess);
    bool onRead(SESSION_t &sess);
    bool onWrite(SESSION_t &sess);

    bool init();
    void teardown();

protected:
    ros::NodeHandle mNh;
    ros::ServiceClient mSrv_Info;

    bool mStopUDS;
    int mEpollfd = 0;
    int mUdsSoc = 0;
};

} // namespace interface
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_UDS_H_
