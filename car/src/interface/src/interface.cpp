#include "interface.h"

#include "rapidjson/writer.h"

using namespace std;
using namespace rapidjson;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace interface
{

bool Interface::onSignaling(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    const char *cmd = sig[RCMP::FIELD_CMD].GetString();

    if (strcasecmp(cmd, RCMP::SIG_LOGIN_RESP))
    {
        // In current version of code, skip signaling SIG_LOGIN_RESP.
        ROS_DEBUG("Debug: In current version of code, skip signaling SIG_LOGIN_RESP.");
        return true;
    }
    else if (strcasecmp(cmd, RCMP::SIG_PING))
    {
        return onSigPing(sess, sig);
    }
    else if (strcasecmp(cmd, RCMP::SIG_PONG))
    {
        return onSigPong(sess, sig);
    }
    else if (strcasecmp(cmd, RCMP::SIG_LOGIN))
    {
        // these signaling should not be received
        ROS_ERROR("Err: wrong signaling(cmd: [%s]", cmd);
        return false;
    }

    // unsupported singaling
    ROS_ERROR("Err: unsupported signaling(cmd: [%s]", cmd);
    return false;
}

bool Interface::onSigPing(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    mRcmp.convertToPong(sig);

    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    sig.Accept(writer);
    const char *payload = buffer.GetString();

    // send signaling via append it to send buffer
    return sendSignaling(sess, buffer);
}

bool Interface::onSigPong(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    // In current version of code, we just skip signaling PONG.
    ROS_DEBUG("Debug: In current version of code, skip signaling PONG.");
    return true;
}

bool Interface::sendSignaling(UDS::SESSION_t &sess, StringBuffer &buffer)
{
    auto len = buffer.GetSize();

    // check empty send buffer size
    if (RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd < len)
    {
        if (sess.sendBufPos == 0)
        {
            // send buffer full
            ROS_DEBUG("Debug: send buffer fulll(soc: [%d]", sess.soc);
            return false;
        }

        // defrag buffer
        int bufLen = sess.sendBufEnd - sess.sendBufPos;
        memmove(sess.sendBuf, sess.sendBuf + sess.sendBufPos, bufLen);
        sess.sendBufPos = 0;
        sess.sendBufEnd = bufLen;

        if (RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd < len)
        {
            // buffer still full after defrag send buffer
            ROS_DEBUG("Debug: send buffer fulll(soc: [%d]", sess.soc);
            return false;
        }
    }

    memcpy(sess.sendBuf + sess.sendBufEnd, buffer.GetString(), len);
    sess.sendBufEnd += len;
}

} // namespace interface
} // namespace car
} // namespace roscar
