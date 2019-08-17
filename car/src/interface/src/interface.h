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
    static bool onSignaling(UDS::SESSION_t &sess, rapidjson::Document &sig);

    // init Interface runtime environment
    static void init(ros::NodeHandle &nh);

protected:
    static bool onSigPing(UDS::SESSION_t &sess, rapidjson::Document &sig);
    static bool onSigPong(UDS::SESSION_t &sess, rapidjson::Document &sig);
    static bool onSigInfo(UDS::SESSION_t &sess, rapidjson::Document &sig);
    static bool onSigMove(UDS::SESSION_t &sess, rapidjson::Document &sig);

    static bool sendToBuf(UDS::SESSION_t &sess, rapidjson::Document &sig);

    static ros::ServiceClient gSvrInfo;
    static ros::ServiceClient gSvrMove;
};

} // namespace interface
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_INTERFACE_H_
