#include "interface.h"

#include "rapidjson/writer.h"

#include "roscar_common/rcmp.h"

using namespace std;
using namespace rapidjson;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace interface
{

std::map<const char *, Interface::FUNC_ONSIG> Interface::mSigFuncMap;

void Interface::init()
{
    mSigFuncMap[RCMP::SIG_PING] = &onSigPing;
    mSigFuncMap[RCMP::SIG_PONG] = &onSigPong;
    mSigFuncMap[RCMP::SIG_INFO] = &onSigInfo;
}

bool Interface::onSignaling(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    const char *cmd = sig[RCMP::FIELD_CMD].GetString();

    auto onSigFuncPare = mSigFuncMap.find(cmd);
    if (onSigFuncPare == mSigFuncMap.end())
    {
        // unsupported signaling
        ROS_ERROR("Err: unsupported signaling(cmd: [%s])", cmd);
        return false;
    }

    auto onSigFunc = onSigFuncPare->second;
    return onSigFunc(sess, sig);
}

bool Interface::onSigPing(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    return sendSignaling(sess, RCMP::convertToPong(sig));
}

bool Interface::onSigPong(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    ROS_DEBUG("Debug: recv PONG.");
    return true;
}

bool Interface::onSigInfo(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    // TODO: ...
    return false;
}

bool Interface::sendSignaling(UDS::SESSION_t &sess, rapidjson::Document &sig)
{
    // convert signaling object into jsong string
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    sig.Accept(writer);

    // check empty send buffer size
    auto len = buffer.GetSize();
    if (RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd < len)
    {
        if (sess.sendBufPos == 0 ||
            (RCMP::RCMP_MAXPAYLOAD - sess.sendBufEnd + sess.sendBufPos < len))
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
    }

    memcpy(sess.sendBuf + sess.sendBufEnd, buffer.GetString(), len);
    sess.sendBufEnd += len;

    return true;
}

} // namespace interface
} // namespace car
} // namespace roscar
