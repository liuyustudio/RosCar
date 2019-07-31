
#include <cstdlib>
#include <string>
#include <thread>
#include "ros/ros.h"

#include "uds.h"
#include "roscar_common/rcmp.h"

using namespace std;
using namespace roscar::car::interface;
using namespace roscar::car::roscar_common;

void udsThreadFunc(const char *uri)
{
    ROS_INFO("start Unix Domain Socket on %s", uri);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface");
    if (argc != 1)
    {
        ROS_INFO("usage: interface");
        return 1;
    }

    UDS uds;
    RCMP rcmp;

    thread udsThread = thread(&UDS::threadFunc, &uds);
    udsThread.join();

    return 0;
}
