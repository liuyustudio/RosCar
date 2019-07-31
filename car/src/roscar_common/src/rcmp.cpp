#include "rcmp.h"
#include <string>
#include <exception>
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

const char *RCMP::SIG_LOGIN = "Login";
const char *RCMP::SIG_LOGIN_RESP = "LoginResp";
const char *RCMP::SIG_LOGOUT = "Logout";
const char *RCMP::SIG_LOGOUT_RESP = "LogoutResp";
const char *RCMP::SIG_PING = "Ping";
const char *RCMP::SIG_PONG = "Pong";
const char *RCMP::SIG_CONTROL = "Ctl";
const char *RCMP::SIG_CONTROL_RESP = "CtlResp";
const char *RCMP::SIG_MT = "Mt";
const char *RCMP::SIG_MT_RESP = "MtResp";
const char *RCMP::SIG_REPORT = "Report";
const char *RCMP::SIG_REPORT_RESP = "ReportResp";

const char *RCMP::SCHEMA_SIG = ""
                               "{"
                               "   \"type\":\"object\","
                               "   \"properties\":{"
                               "       \"cmd\":{"
                               "           \"type\":\"string\""
                               "       },"
                               "       \"seq\":{"
                               "           \"type\":\"integer\","
                               "           \"minimum\":0"
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
                                     "           \"type\":\"integer\","
                                     "           \"minimum\":0"
                                     "       }"
                                     "       \"payload\":{"
                                     "          \"type\":\"object\","
                                     "          \"properties\":{"
                                     "              \"ver\":{"
                                     "                  \"type\":\"integer\","
                                     "                  \"minimum\":0"
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
                                          "           \"type\":\"integer\","
                                          "           \"minimum\":0"
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
                                      "           \"type\":\"integer\","
                                      "           \"minimum\":0"
                                      "       }"
                                      "   },"
                                      "   \"required\":[\"cmd\",\"seq\"]"
                                      "}";

const char *RCMP::SCHEMA_SIG_LOGOUT_RESP = RCMP::SCHEMA_SIG_LOGIN_RESP;

const char *RCMP::SCHEMA_SIG_PING = RCMP::SCHEMA_SIG_LOGOUT;
const char *RCMP::SCHEMA_SIG_PONG = RCMP::SCHEMA_SIG_LOGIN_RESP;

RCMP::RCMP()
{
    // create schemas
    mpSchema_Sig = loadSchema(SCHEMA_SIG);
    mpSchema_SigLogin = loadSchema(SCHEMA_SIG_LOGIN);
    mpSchema_SigLoginResp = loadSchema(SCHEMA_SIG_LOGIN_RESP);
    mpSchema_SigLogout = loadSchema(SCHEMA_SIG_LOGOUT);
    mpSchema_SigLogoutResp = loadSchema(SCHEMA_SIG_LOGOUT_RESP);
    mpSchema_SigPing = loadSchema(SCHEMA_SIG_PING);
    mpSchema_SigPong = loadSchema(SCHEMA_SIG_PONG);

    // create validators
    mpSchemaValidator_Sig = new SchemaValidator(*mpSchema_Sig);
    mpSchemaValidator_SigLogin = new SchemaValidator(*mpSchema_SigLogin);
    mpSchemaValidator_SigLoginResp = new SchemaValidator(*mpSchema_SigLoginResp);
    mpSchemaValidator_SigLogout = new SchemaValidator(*mpSchema_SigLogout);
    mpSchemaValidator_SigLogoutResp = new SchemaValidator(*mpSchema_SigLogoutResp);
    mpSchemaValidator_SigPing = new SchemaValidator(*mpSchema_SigPing);
    mpSchemaValidator_SigPong = new SchemaValidator(*mpSchema_SigPong);
}

RCMP::~RCMP()
{
    // release validators
    delete mpSchemaValidator_Sig;
    delete mpSchemaValidator_SigLogin;
    delete mpSchemaValidator_SigLoginResp;
    delete mpSchemaValidator_SigLogout;
    delete mpSchemaValidator_SigLogoutResp;
    delete mpSchemaValidator_SigPing;
    delete mpSchemaValidator_SigPong;

    // release schemas
    delete mpSchema_Sig;
    delete mpSchema_SigLogin;
    delete mpSchema_SigLoginResp;
    delete mpSchema_SigLogout;
    delete mpSchema_SigLogoutResp;
    delete mpSchema_SigPing;
    delete mpSchema_SigPong;
}

SchemaDocument *RCMP::loadSchema(const char *schema)
{
    Document doc;
    if (doc.Parse(SCHEMA_SIG).HasParseError())
    {
        throw invalid_argument((string("SCHEMA_SIG not valid: ") + schema));
    }

    return new SchemaDocument(doc);
}

int RCMP::parse(void *pBuf, int &len, rapidjson::Document &doc)
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

bool RCMP::verifySig(rapidjson::Document &doc)
{
    if (!doc.Accept(*mpSchemaValidator_Sig))
    {
        // invalid format
        return ERROR_INVALID;
    }

    // login
    if (strcmp(doc[FIELD_CMD].GetString(), SIG_LOGIN) == 0)
    {
        if (!doc.Accept(*mpSchemaValidator_SigLogin))
        {
            return ERROR_INVALID;
        }
    }
    // login resp
    else if (strcmp(doc[FIELD_CMD].GetString(), SIG_LOGIN_RESP) == 0)
    {
        if (!doc.Accept(*mpSchemaValidator_SigLoginResp))
        {
            return ERROR_INVALID;
        }
    }

    // logout
    else if (strcmp(doc[FIELD_CMD].GetString(), SIG_LOGOUT) == 0)
    {
        if (!doc.Accept(*mpSchemaValidator_SigLogout))
        {
            return ERROR_INVALID;
        }
    }
    // logout resp
    else if (strcmp(doc[FIELD_CMD].GetString(), SIG_LOGOUT_RESP) == 0)
    {
        if (!doc.Accept(*mpSchemaValidator_SigLogoutResp))
        {
            return ERROR_INVALID;
        }
    }

    // ping
    else if (strcmp(doc[FIELD_CMD].GetString(), SIG_PING) == 0)
    {
        if (!doc.Accept(*mpSchemaValidator_SigPing))
        {
            return ERROR_INVALID;
        }
    }
    // pong resp
    else if (strcmp(doc[FIELD_CMD].GetString(), SIG_PONG) == 0)
    {
        if (!doc.Accept(*mpSchemaValidator_SigPong))
        {
            return ERROR_INVALID;
        }
    }

    return SUCCESS;
}

} // namespace roscar_common
} // namespace car
} // namespace roscar
