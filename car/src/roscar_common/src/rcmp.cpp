#include "rcmp.h"
#include <exception>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "rapidjson/writer.h"
#include "rapidjson/error/en.h"
#include "const.h"
#include "error.h"

#include "schemaSig.cpp"

using namespace std;
using namespace rapidjson;

namespace roscar
{
namespace car
{
namespace roscar_common
{

const char *RCMP::FIELD_CMD = "cmd";
const char *RCMP::FIELD_SEQ = "seq";
const char *RCMP::FIELD_ERRNO = "errno";
const char *RCMP::FIELD_ERRMSG = "errmsg";
const char *RCMP::FIELD_PAYLOAD = "payload";
const char *RCMP::FIELD_LOGIN_VER = "ver";
const char *RCMP::FIELD_LOGIN_TYPE = "type";
const char *RCMP::FIELD_LOGIN_ID = "id";
const char *RCMP::FIELD_INFORESP_ID = "id";
const char *RCMP::FIELD_INFORESP_TYPE = "type";
const char *RCMP::FIELD_INFORESP_NAME = "name";
const char *RCMP::FIELD_MOVE_ANGLE = "angle";
const char *RCMP::FIELD_MOVE_POWER = "power";
const char *RCMP::FIELD_MOVE_DURATION = "duration";

const char *RCMP::SIGNALING = "Signaling";
const char *RCMP::SIG_LOGIN = "Login";
const char *RCMP::SIG_LOGIN_RESP = "LoginResp";
const char *RCMP::SIG_LOGOUT = "Logout";
const char *RCMP::SIG_LOGOUT_RESP = "LogoutResp";
const char *RCMP::SIG_PING = "Ping";
const char *RCMP::SIG_PONG = "Pong";
const char *RCMP::SIG_INFO = "Info";
const char *RCMP::SIG_INFO_RESP = "InfoResp";
const char *RCMP::SIG_MOVE = "Move";
const char *RCMP::SIG_MOVE_RESP = "MoveResp";

std::map<const char *, const char *> RCMP::gSigRespCmdMap;
std::map<const char *, rapidjson::SchemaDocument *> RCMP::gSchemaMap;
std::map<const char *, rapidjson::SchemaValidator *> RCMP::gValidatorMap;

RCMP::Initializer::Initializer()
{
    RCMP::init();
}

RCMP::Initializer RCMP::gInitializer;

int RCMP::parse(void *pBuf, int &len, Document &doc)
{
    FRAME_t *pFrame = static_cast<FRAME_t *>(pBuf);
    int nRet = verifyFrame(pFrame, len);
    if (nRet != SUCCESS)
    {
        ROS_ERROR("[RCMP::parse] verify frame fail: %d", nRet);
        return nRet;
    }

    string strPayload;

    // set 'len' to frame size
    len = pFrame->len();
    strPayload.append(pFrame->payload, len - RCMP_FRAMEHEADSIZE);

    doc.Parse(strPayload.c_str());
    if (doc.HasParseError())
    {
        // json parse fail
        ROS_ERROR("[RCMP::parse] parse err[offset %u]: %s\n%s",
                  (unsigned)doc.GetErrorOffset(),
                  GetParseError_En(doc.GetParseError()),
                  strPayload.c_str());
        return ERROR_INVALID;
    }
    if (!verifySig(doc))
    {
        // invalid signaling
        ROS_ERROR("[RCMP::parse] invalid signaling");
        return ERROR_INVALID;
    }

    return SUCCESS;
}

const char *RCMP::getRespCmd(const char *cmd)
{
    for (auto &item : gSigRespCmdMap)
    {
        if (strcasecmp(item.first, cmd))
            continue;

        return item.second;
    }

    ROS_ERROR("[RCMP::getRespCmd] corresponding response of cmd[%s] not found", cmd);
    return nullptr;
}

Document &RCMP::convertToResp(rapidjson::Document &sig,
                              const int err,
                              const char *errmsg,
                              rapidjson::Value *payload)
{
    const char *reqCmd = sig[FIELD_CMD].GetString();
    const char *respCmd = RCMP::getRespCmd(reqCmd);
    assert(respCmd);

    auto &alloc = sig.GetAllocator();

    // change cmd
    sig.RemoveMember(FIELD_CMD);
    sig.AddMember(Value::StringRefType(FIELD_CMD), Value::StringRefType(respCmd), alloc);

    // errno
    if (sig.HasMember(FIELD_ERRNO))
    {
        sig[FIELD_ERRNO].SetInt(err);
    }
    else
    {
        sig.AddMember(Value::StringRefType(FIELD_ERRNO), err, alloc);
    }
    // errmsg
    if (sig.HasMember(FIELD_ERRMSG))
    {
        sig[FIELD_ERRMSG].SetString(Value::StringRefType(errmsg));
    }
    else
    {
        sig.AddMember(Value::StringRefType(FIELD_ERRMSG),
                      Value::StringRefType(errmsg),
                      alloc);
    }

    // payload
    if (payload)
    {
        sig.RemoveMember(FIELD_PAYLOAD);
        sig.AddMember(Value::StringRefType(FIELD_PAYLOAD), *payload, alloc);
    }
    else
    {
        // set 'empty' payload
        sig[FIELD_PAYLOAD].SetNull();
    }

    return sig;
}

string RCMP::getJson(rapidjson::Document &sig)
{
    // convert signaling object to json string
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    sig.Accept(writer);

    return buffer.GetString();
}

int RCMP::fillFrame(void *buf, const int len, const void *payload, const int payloadLen)
{
    assert(len > RCMP_FRAMEHEADSIZE);

    if (payloadLen + RCMP_FRAMEHEADSIZE > len)
    {
        ROS_ERROR("[RCMP::fillFrame] payloadLen[%d] + RCMP_FRAMEHEADSIZE[%d] > len[%d]",
                  payloadLen,
                  RCMP_FRAMEHEADSIZE,
                  len);
        return 0;
    }

    FRAME_t *frame = static_cast<FRAME_t *>(buf);

    frame->init(payload, payloadLen);

    return frame->len();
}

int RCMP::fillFrame(void *buf, const int len, std::string &sig)
{
    return fillFrame(buf, len, sig.c_str(), sig.length());
}

int RCMP::fillFrame(void *buf, const int len, std::string &&sig)
{
    return fillFrame(buf, len, sig);
}

int RCMP::fillFrame(void *buf, const int len, rapidjson::Document &sig)
{
    return fillFrame(buf, len, getJson(sig));
}

void RCMP::init()
{
    // init req->resp cmd maping
    gSigRespCmdMap[SIG_LOGIN] = SIG_LOGIN_RESP;
    gSigRespCmdMap[SIG_LOGOUT] = SIG_LOGOUT_RESP;
    gSigRespCmdMap[SIG_PING] = SIG_PONG;
    gSigRespCmdMap[SIG_INFO] = SIG_INFO_RESP;
    gSigRespCmdMap[SIG_MOVE] = SIG_MOVE_RESP;

    // init schemas and validators
    initSchemaValidator(SIGNALING, SCHEMA_SIG);
    initSchemaValidator(SIG_LOGIN, SCHEMA_SIG_LOGIN);
    initSchemaValidator(SIG_LOGIN_RESP, SCHEMA_SIG_LOGIN_RESP);
    initSchemaValidator(SIG_LOGOUT, SCHEMA_SIG_LOGOUT);
    initSchemaValidator(SIG_LOGOUT_RESP, SCHEMA_SIG_LOGOUT_RESP);
    initSchemaValidator(SIG_PING, SCHEMA_SIG_PING);
    initSchemaValidator(SIG_PONG, SCHEMA_SIG_PONG);
    initSchemaValidator(SIG_INFO, SCHEMA_SIG_INFO);
    initSchemaValidator(SIG_INFO_RESP, SCHEMA_SIG_INFO_RESP);
    initSchemaValidator(SIG_MOVE, SCHEMA_SIG_MOVE);
    initSchemaValidator(SIG_MOVE_RESP, SCHEMA_SIG_MOVE_RESP);
}

void RCMP::initSchemaValidator(const char *name, const char *schema)
{
    Document doc;
    if (doc.Parse(schema).HasParseError())
    {
        std::ostringstream stringStream;
        stringStream << "schema[" << name << "] not valid: " << schema;
        throw invalid_argument(stringStream.str());
    }

    // create schema & validators
    SchemaDocument *pSchema = new SchemaDocument(doc);
    gSchemaMap[name] = pSchema;
    gValidatorMap[name] = new SchemaValidator(*pSchema);
}

int RCMP::verifyFrame(FRAME_t *pFrame, int len)
{
    if (len < RCMP_FRAMEHEADSIZE)
    {
        // Need more data
        ROS_DEBUG("[RCMP::verifyFrame] Need more data");

        return NEED_MORE_DATA;
    }

    // check start flag
    if (pFrame->startFlag != RCMP_STARTFLAG)
    {
        // invalid Start Flag
        ROS_ERROR("[RCMP::verifyFrame] invalid Start Flag");
        return ERROR_INVALID;
    }

    // check signaling size
    int sigLen = pFrame->len();
    if (sigLen > RCMP_MAX_SIGNALING_LENGTH)
    {
        // signaling too large
        ROS_ERROR("[RCMP::verifyFrame] signaling too large");
        return ERROR_INVALID;
    }
    else if (len < sigLen)
    {
        // Need more data
        ROS_DEBUG("[RCMP::verifyFrame] Need more data");
        return NEED_MORE_DATA;
    }

    return SUCCESS;
}

bool RCMP::verifySig(Document &doc)
{
    rapidjson::SchemaValidator *pValidator = nullptr;

    // check signaling's basic format
    pValidator = gValidatorMap[SIGNALING];
    if (!doc.Accept(*pValidator))
    {
        // Input JSON is invalid according to the schema
        // Output diagnostic information
        StringBuffer sb;
        pValidator->GetInvalidSchemaPointer().StringifyUriFragment(sb);
        ROS_ERROR("[RCMP::verifySig] Invalid schema: %s", sb.GetString());
        ROS_ERROR("[RCMP::verifySig] Invalid keyword: %s", pValidator->GetInvalidSchemaKeyword());
        sb.Clear();
        pValidator->GetInvalidDocumentPointer().StringifyUriFragment(sb);
        ROS_ERROR("[RCMP::verifySig] Invalid document: %s", sb.GetString());

        return false;
    }

    const char *cmdField = doc[FIELD_CMD].GetString();

    if (strcasecmp(cmdField, SIG_LOGIN) == 0)
        pValidator = gValidatorMap[SIG_LOGIN];
    else if (strcasecmp(cmdField, SIG_LOGIN_RESP) == 0)
        pValidator = gValidatorMap[SIG_LOGIN_RESP];
    else if (strcasecmp(cmdField, SIG_LOGOUT) == 0)
        pValidator = gValidatorMap[SIG_LOGOUT];
    else if (strcasecmp(cmdField, SIG_LOGOUT_RESP) == 0)
        pValidator = gValidatorMap[SIG_LOGOUT_RESP];
    else if (strcasecmp(cmdField, SIG_PING) == 0)
        pValidator = gValidatorMap[SIG_PING];
    else if (strcasecmp(cmdField, SIG_PONG) == 0)
        pValidator = gValidatorMap[SIG_PONG];
    else if (strcasecmp(cmdField, SIG_INFO) == 0)
        pValidator = gValidatorMap[SIG_INFO];
    else if (strcasecmp(cmdField, SIG_INFO_RESP) == 0)
        pValidator = gValidatorMap[SIG_INFO_RESP];
    else if (strcasecmp(cmdField, SIG_MOVE) == 0)
        pValidator = gValidatorMap[SIG_MOVE];
    else if (strcasecmp(cmdField, SIG_MOVE_RESP) == 0)
        pValidator = gValidatorMap[SIG_MOVE_RESP];
    else
        return false; // unknown signaling cmd

    // verify format
    assert(pValidator);
    if (doc.Accept(*pValidator))
    {
        return true;
    }
    else
    {
        // Input JSON is invalid according to the schema
        // Output diagnostic information
        StringBuffer sb;
        pValidator->GetInvalidSchemaPointer().StringifyUriFragment(sb);
        ROS_ERROR("[RCMP::verifySig] Invalid schema: %s", sb.GetString());
        ROS_ERROR("[RCMP::verifySig] Invalid keyword: %s", pValidator->GetInvalidSchemaKeyword());
        sb.Clear();
        pValidator->GetInvalidDocumentPointer().StringifyUriFragment(sb);
        ROS_ERROR("[RCMP::verifySig] Invalid document: %s", sb.GetString());

        return false;
    }
}

} // namespace roscar_common
} // namespace car
} // namespace roscar
