#include "interface.h"

#include "roscar_common/rcmp.h"
#include "pilot/Info.h"

using namespace std;
using namespace rapidjson;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace interface
{

ros::ServiceClient Interface::gSvrInfo;

void Interface::init(ros::NodeHandle &nh)
{
    gSvrInfo = nh.serviceClient<pilot::Info>("info");
}

bool Interface::onSignaling(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    const char *cmd = sig[RCMP::FIELD_CMD].GetString();

    if (strcasecmp(cmd, RCMP::SIG_PING) == 0)
        return onSigPing(sess, sig);
    else if (strcasecmp(cmd, RCMP::SIG_PONG) == 0)
        return onSigPong(sess, sig);
    else if (strcasecmp(cmd, RCMP::SIG_INFO) == 0)
        return onSigInfo(sess, sig);
    else
        return false; // unknown signaling cmd
}

bool Interface::onSigPing(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    return sendSignaling(sess, RCMP::convertToResp(sig));
}

bool Interface::onSigPong(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    ROS_DEBUG("Debug: recv PONG.");
    return true;
}

bool Interface::onSigInfo(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    pilot::Info info;
    if (!gSvrInfo.call(info))
    {
        ROS_ERROR("Fail to call service: [pilot::Info]");
        return false;
    }

    auto &alloc = sig.GetAllocator();
    Value payload(kObjectType);
    payload.AddMember(Value::StringRefType(RCMP::FIELD_INFORESP_ID),
                      Value::StringRefType(info.response.id.c_str()),
                      alloc);
    payload.AddMember(Value::StringRefType(RCMP::FIELD_INFORESP_TYPE),
                      Value::StringRefType(info.response.type.c_str()),
                      alloc);
    payload.AddMember(Value::StringRefType(RCMP::FIELD_INFORESP_NAME),
                      Value::StringRefType(info.response.name.c_str()),
                      alloc);

    return sendSignaling(sess, RCMP::convertToResp(sig, &payload));
}

bool Interface::sendSignaling(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    // wrap signaling in RCMPFrame, then put it into sending buffer
    int nRet = RCMP::fillFrame(sess.sendBuf + sess.sendBufEnd,
                               RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd,
                               sig);
    if (!nRet)
    {
        ROS_ERROR("Fail to wrap signaling in RCMPFrame, then put it into sending buffer. Error: %d", nRet);
        return false;
    }

    return true;
}

} // namespace interface
} // namespace car
} // namespace roscar
