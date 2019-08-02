#include "rcmp.h"
#include <exception>
#include <sstream>
#include <string>
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
const char *RCMP::FIELD_PAYLOAD = "payload";
const char *RCMP::FIELD_LOGIN_VER = "ver";
const char *RCMP::FIELD_LOGIN_TYPE = "type";
const char *RCMP::FIELD_LOGIN_ID = "id";

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
                                     "       }"
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
                                          "       }"
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
                                         "       }"
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

RCMP::RCMP()
{
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

RCMP::~RCMP()
{
    // release validators
    for (auto item : mValidatorMap)
    {
        delete item.second;
    }

    // release schemas
    for (auto item : mSchemaMap)
    {
        delete item.second;
    }
}

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
    }
    catch (...)
    {
        return ERROR_INVALID;
    }
}

void RCMP::convertToPong(Document &sig)
{
    auto &alloc = sig.GetAllocator();

    sig["cmd"].SetString("PONG");
    sig.AddMember("errno", 0, alloc);
    sig.AddMember("errmsg", "", alloc);

    // add 'empty' payload
    rapidjson::Value payload(rapidjson::kNullType);
    sig.AddMember("payload", payload, alloc);
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
    mSchemaMap[name] = pSchema;
    mValidatorMap[name] = new SchemaValidator(*pSchema);
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
    else if (sigLen < RCMP_MAX_SIGNALING_LENGTH)
    {
        // Need more data
        return NEED_MORE_DATA;
    }

    return SUCCESS;
}

bool RCMP::verifySig(Document &doc)
{
    // check signaling's basic format
    if (!doc.Accept(*mValidatorMap[SIGNALING]))
    {
        // invalid format
        return ERROR_INVALID;
    }

    const char * sigCmd = NULL;

    // login
    if (strcasecmp(doc[FIELD_CMD].GetString(), SIG_LOGIN) == 0)
    {
        sigCmd = SIG_LOGIN;
    }
    // login resp
    else if (strcasecmp(doc[FIELD_CMD].GetString(), SIG_LOGIN_RESP) == 0)
    {
        sigCmd = SIG_LOGIN_RESP;
    }

    // logout
    else if (strcasecmp(doc[FIELD_CMD].GetString(), SIG_LOGOUT) == 0)
    {
        sigCmd = SIG_LOGOUT;
    }
    // logout resp
    else if (strcasecmp(doc[FIELD_CMD].GetString(), SIG_LOGOUT_RESP) == 0)
    {
        sigCmd = SIG_LOGOUT_RESP;
    }

    // ping
    else if (strcasecmp(doc[FIELD_CMD].GetString(), SIG_PING) == 0)
    {
        sigCmd = SIG_PING;
    }
    // pong resp
    else if (strcasecmp(doc[FIELD_CMD].GetString(), SIG_PONG) == 0)
    {
        sigCmd = SIG_PONG;
    }

    if (sigCmd)
    {
        return doc.Accept(*mValidatorMap[sigCmd]) ? SUCCESS : ERROR_INVALID;
    }
    else
    {
        // unknown signaling cmd
        return ERROR;
    }
}

} // namespace roscar_common
} // namespace car
} // namespace roscar
