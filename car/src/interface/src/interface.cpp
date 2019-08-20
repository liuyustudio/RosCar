#include "interface.h"

#include "roscar_common/rcmp.h"
#include "pilot/Info.h"
#include "pilot/Move.h"

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
ros::ServiceClient Interface::gSvrMove;

void Interface::init(ros::NodeHandle &nh)
{
    gSvrInfo = nh.serviceClient<pilot::Info>("info");
    gSvrMove = nh.serviceClient<pilot::Move>("move");
}

bool Interface::onSignaling(UDS::BufType &buffer, rapidjson::Document &sig)
{
    const char *cmd = sig[RCMP::FIELD_CMD].GetString();

    if (strcasecmp(cmd, RCMP::SIG_PING) == 0)
        return onSigPing(buffer, sig);
    else if (strcasecmp(cmd, RCMP::SIG_PONG) == 0)
        return onSigPong(buffer, sig);
    else if (strcasecmp(cmd, RCMP::SIG_INFO) == 0)
        return onSigInfo(buffer, sig);
    else if (strcasecmp(cmd, RCMP::SIG_MOVE) == 0)
        return onSigMove(buffer, sig);
    else
        return false; // unknown signaling cmd
}

bool Interface::onSigPing(UDS::BufType &buffer, rapidjson::Document &sig)
{
    return sendToBuf(buffer, RCMP::convertToResp(sig));
}

bool Interface::onSigPong(UDS::BufType &buffer, rapidjson::Document &sig)
{
    ROS_DEBUG("[Interface::onSigPong] recv PONG.");
    return true;
}

bool Interface::onSigInfo(UDS::BufType &buffer, rapidjson::Document &sig)
{
    pilot::Info info;
    if (!gSvrInfo.call(info))
    {
        ROS_ERROR("[Interface::onSigInfo] Fail to call service: [pilot::Info]");
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

    return sendToBuf(buffer, RCMP::convertToResp(sig, &payload));
}

bool Interface::onSigMove(UDS::BufType &buffer, rapidjson::Document &sig)
{
    pilot::Move move;

    move.request.angle = sig[RCMP::FIELD_PAYLOAD][RCMP::FIELD_MOVE_ANGLE].GetFloat();
    move.request.power = sig[RCMP::FIELD_PAYLOAD][RCMP::FIELD_MOVE_POWER].GetFloat();
    move.request.duration = sig[RCMP::FIELD_PAYLOAD][RCMP::FIELD_MOVE_DURATION].GetFloat();

    if (!gSvrMove.call(move))
    {
        ROS_ERROR("[Interface::onSigMove] Fail to call service: [pilot::Move]");
        return false;
    }

    return sendToBuf(buffer,
                     RCMP::convertToResp(sig,
                                         move.response.err_code,
                                         move.response.err_msg.c_str()));
}

bool Interface::sendToBuf(UDS::BufType &buffer, rapidjson::Document &sig)
{
    char *p;
    int len;
    tie(p, len) = buffer.getSendBuf();

    // wrap signaling in RCMPFrame, then put it into sending buffer
    int nRet = RCMP::fillFrame(p, len, sig);
    if (0 == nRet)
    {
        ROS_ERROR("[Interface::sendToBuf] fill buffer fail.");
        return false;
    }
    buffer.incSendEnd(nRet);

    return true;
}

} // namespace interface
} // namespace car
} // namespace roscar
