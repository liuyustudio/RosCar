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
    static const int EPOLL_TIMEOUT;
    static const int EPOLL_RETRY_INTERVAL;
    static const int MAX_CLIENT_COUNT;
    static const char *UDS_PATH;

public:
    UDS();

protected:
    static void threadFunc(UDS * pUDS);

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
