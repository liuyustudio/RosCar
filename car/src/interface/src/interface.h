#ifndef _ROSCAR_CAR_INTERFACE_INTERFACE_H_
#define _ROSCAR_CAR_INTERFACE_INTERFACE_H_

#include <map>
#include "rapidjson/document.h"
#include "ros/ros.h"
#include "uds.h"

namespace roscar
{
namespace car
{
namespace interface
{

class Interface
{
protected:
    Interface(){};

public:
    // overide
    static bool onSignaling(UDS::BufType &buffer, rapidjson::Document &sig);

    // init Interface runtime environment
    static void init(ros::NodeHandle &nh);

protected:
    static bool onSigPing(UDS::BufType &buffer, rapidjson::Document &sig);
    static bool onSigPong(UDS::BufType &buffer, rapidjson::Document &sig);
    static bool onSigInfo(UDS::BufType &buffer, rapidjson::Document &sig);
    static bool onSigMove(UDS::BufType &buffer, rapidjson::Document &sig);

    static bool sendToBuf(UDS::BufType &buffer, rapidjson::Document &sig);

    static ros::ServiceClient gSvrInfo;
    static ros::ServiceClient gSvrMove;
};

} // namespace interface
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_INTERFACE_H_
