#include "rcmp.h"
#include <exception>
#include <sstream>
#include <string>
#include "rapidjson/writer.h"
#include "const.h"
#include "error.h"

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

const char *RCMP::SIGNALING = "Signaling";
const char *RCMP::SIG_LOGIN = "Login";
const char *RCMP::SIG_LOGIN_RESP = "LoginResp";
const char *RCMP::SIG_LOGOUT = "Logout";
const char *RCMP::SIG_LOGOUT_RESP = "LogoutResp";
const char *RCMP::SIG_PING = "Ping";
const char *RCMP::SIG_PONG = "Pong";
const char *RCMP::SIG_INFO = "Info";
const char *RCMP::SIG_INFO_RESP = "InfoResp";

const char *RCMP::SCHEMA_SIG = ""
                               "{"
                               "   \"type\":\"object\","
                               "   \"properties\":{"
                               "       \"cmd\":{"
                               "           \"type\":\"string\""
                               "       },"
                               "       \"seq\":{"
                               "           \"type\":\"integer\""
                               "       }"
                               "   },"
                               "   \"required\":[\"cmd\",\"seq\"]"
                               "}";

const char *RCMP::SCHEMA_SIG_LOGIN = ""
                                     "{"
                                     "   \"type\":\"object\","
                                     "   \"properties\":{"
                                     "       \"cmd\":{"
                                     "           \"type\":\"string\""
                                     "       },"
                                     "       \"seq\":{"
                                     "           \"type\":\"integer\""
                                     "       },"
                                     "       \"payload\":{"
                                     "          \"type\":\"object\","
                                     "          \"properties\":{"
                                     "              \"ver\":{"
                                     "                  \"type\":\"integer\""
                                     "              },"
                                     "              \"type\":{"
                                     "                  \"type\":\"string\""
                                     "              },"
                                     "              \"id\":{"
                                     "                  \"type\":\"string\""
                                     "              }"
                                     "          },"
                                     "          \"required\":[\"ver\",\"type\",\"id\"]"
                                     "       }"
                                     "   },"
                                     "   \"required\":[\"cmd\",\"seq\",\"payload\"]"
                                     "}";

const char *RCMP::SCHEMA_SIG_LOGIN_RESP = ""
                                          "{"
                                          "   \"type\":\"object\","
                                          "   \"properties\":{"
                                          "       \"cmd\":{"
                                          "           \"type\":\"string\""
                                          "       },"
                                          "       \"seq\":{"
                                          "           \"type\":\"integer\""
                                          "       },"
                                          "       \"errno\":{"
                                          "           \"type\":\"integer\""
                                          "       },"
                                          "       \"errmsg\":{"
                                          "           \"type\":\"string\""
                                          "       },"
                                          "       \"payload\":{"
                                          "           \"type\":\"object\""
                                          "       }"
                                          "   },"
                                          "   \"required\":[\"cmd\",\"seq\",\"errno\",\"errmsg\",\"payload\"]"
                                          "}";

const char *RCMP::SCHEMA_SIG_LOGOUT = ""
                                      "{"
                                      "   \"type\":\"object\","
                                      "   \"properties\":{"
                                      "       \"cmd\":{"
                                      "           \"type\":\"string\""
                                      "       },"
                                      "       \"seq\":{"
                                      "           \"type\":\"integer\""
                                      "       }"
                                      "   },"
                                      "   \"required\":[\"cmd\",\"seq\"]"
                                      "}";

const char *RCMP::SCHEMA_SIG_LOGOUT_RESP = RCMP::SCHEMA_SIG_LOGIN_RESP;

const char *RCMP::SCHEMA_SIG_PING = RCMP::SCHEMA_SIG_LOGOUT;
const char *RCMP::SCHEMA_SIG_PONG = RCMP::SCHEMA_SIG_LOGOUT_RESP;

const char *RCMP::SCHEMA_SIG_INFO = RCMP::SCHEMA_SIG_LOGOUT;
const char *RCMP::SCHEMA_SIG_INFO_RESP = ""
                                         "{"
                                         "   \"type\":\"object\","
                                         "   \"properties\":{"
                                         "       \"cmd\":{"
                                         "           \"type\":\"string\""
                                         "       },"
                                         "       \"seq\":{"
                                         "           \"type\":\"integer\""
                                         "       },"
                                         "       \"payload\":{"
                                         "          \"type\":\"object\","
                                         "          \"properties\":{"
                                         "              \"id\":{"
                                         "                  \"type\":\"string\""
                                         "              },"
                                         "              \"type\":{"
                                         "                  \"type\":\"string\""
                                         "              },"
                                         "              \"name\":{"
                                         "                  \"type\":\"string\""
                                         "              }"
                                         "          },"
                                         "          \"required\":[\"id\",\"type\",\"name\"]"
                                         "       }"
                                         "   },"
                                         "   \"required\":[\"cmd\",\"seq\",\"payload\"]"
                                         "}";

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
        return nRet;
    }

    try
    {
        string strPayload;

        // set 'len' to frame size
        len = pFrame->len();
        strPayload.append(pFrame->payload, len - RCMP_FRAMEHEADSIZE);

        doc.Parse(strPayload.c_str());
        if (doc.HasParseError())
        {
            // parse payload, as Json, fail
            return ERROR_INVALID;
        }
        if (!verifySig(doc))
        {
            // invalid signaling
            return ERROR_INVALID;
        }

        return SUCCESS;
    }
    catch (...)
    {
        return ERROR_INVALID;
    }
}

const char *RCMP::getRespCmd(const char *cmd)
{
    auto respCmd = gSigRespCmdMap.find(cmd);
    return (respCmd != gSigRespCmdMap.end()) ? respCmd->second : nullptr;
}

Document &RCMP::convertToResp(rapidjson::Document &sig,
                              const int err,
                              const char *errmsg,
                              rapidjson::Value *payload)
{
    const char *respCmd = RCMP::getRespCmd(sig[FIELD_CMD].GetString());
    assert(respCmd);

    auto &alloc = sig.GetAllocator();

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
    string jsonStr = getJson(sig);
    return fillFrame(buf, len, getJson(sig));
}

void RCMP::init()
{
    // init req->resp cmd maping
    gSigRespCmdMap[SIG_LOGIN] = SIG_LOGIN_RESP;
    gSigRespCmdMap[SIG_LOGOUT] = SIG_LOGOUT_RESP;
    gSigRespCmdMap[SIG_PING] = SIG_PONG;
    gSigRespCmdMap[SIG_INFO] = SIG_INFO_RESP;

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
        return NEED_MORE_DATA;
    }

    // check start flag
    if (pFrame->startFlag != RCMP_STARTFLAG)
    {
        // invalid Start Flag
        return ERROR_INVALID;
    }

    // check signaling size
    int sigLen = pFrame->len();
    if (sigLen > RCMP_MAX_SIGNALING_LENGTH)
    {
        // too large
        return ERROR_INVALID;
    }
    else if (len < sigLen)
    {
        // Need more data
        return NEED_MORE_DATA;
    }

    return SUCCESS;
}

bool RCMP::verifySig(Document &doc)
{
    // check signaling's basic format
    if (!doc.Accept(*gValidatorMap[SIGNALING]))
    {
        // invalid format
        return false;
    }

    rapidjson::SchemaValidator *pValidator = nullptr;
    const char *cmdField = doc[FIELD_CMD].GetString();

    if (strcasecmp(cmdField, SIG_LOGIN) == 0)
        pValidator = gValidatorMap[SIG_LOGIN];
    else if (strcasecmp(cmdField, SIG_LOGIN_RESP) == 0)
        pValidator = gValidatorMap[SIG_LOGIN];
    else if (strcasecmp(cmdField, SIG_LOGOUT) == 0)
        pValidator = gValidatorMap[SIG_LOGIN];
    else if (strcasecmp(cmdField, SIG_LOGOUT_RESP) == 0)
        pValidator = gValidatorMap[SIG_LOGIN];
    else if (strcasecmp(cmdField, SIG_PING) == 0)
        pValidator = gValidatorMap[SIG_LOGIN];
    else if (strcasecmp(cmdField, SIG_PONG) == 0)
        pValidator = gValidatorMap[SIG_LOGIN];
    else if (strcasecmp(cmdField, SIG_INFO) == 0)
        pValidator = gValidatorMap[SIG_LOGIN];
    else if (strcasecmp(cmdField, SIG_INFO_RESP) == 0)
        pValidator = gValidatorMap[SIG_LOGIN];
    else
        return false; // unknown signaling cmd

    // TODO: Debug HERE...

    // verify format
    assert(pValidator);
    return doc.Accept(*pValidator) ? true : false;
}

} // namespace roscar_common
} // namespace car
} // namespace roscar
