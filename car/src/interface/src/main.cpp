
#include <cstdlib>
#include <string>
#include <thread>

#include "ros/ros.h"
#include "interface.h"
#include "uds.h"

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

    ros::NodeHandle nh;
    ROS_INFO("ROS Car Interface Ready.");

    Interface::init(nh);
    UDS uds(Interface::onSignaling);

    thread udsThread = thread(&UDS::threadFunc, &uds);
    udsThread.join();

    return 0;
}
