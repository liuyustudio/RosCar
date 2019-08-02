#ifndef _ROSCAR_CAR_ROSCARCOMMON_RCMP_H_
#define _ROSCAR_CAR_ROSCARCOMMON_RCMP_H_

#include <assert.h>
#include <map>

#include "rapidjson/document.h"
#include "rapidjson/schema.h"
#include "rapidjson/stringbuffer.h"

#include "roscar_common/error.h"

namespace roscar
{
namespace car
{
namespace roscar_common
{

class RCMP
{
public:
    static const int RCMP_FRAMEHEADSIZE = 4;
    static const char RCMP_STARTFLAG = 0x5A;
    static const int RCMP_VERSION = 1;
    static const int RCMP_MIN_FRAME_SIZE = RCMP_FRAMEHEADSIZE;
    static const int RCMP_MAXPAYLOAD = 0x10000 - RCMP_FRAMEHEADSIZE;

    static const char *FIELD_CMD;
    static const char *FIELD_SEQ;
    static const char *FIELD_ERRNO;
    static const char *FIELD_ERRMSG;
    static const char *FIELD_PAYLOAD;
    static const char *FIELD_LOGIN_VER;
    static const char *FIELD_LOGIN_TYPE;
    static const char *FIELD_LOGIN_ID;
    static const char *FIELD_INFORESP_ID;
    static const char *FIELD_INFORESP_TYPE;
    static const char *FIELD_INFORESP_NAME;

    static const char *SIGNALING;
    static const char *SIG_LOGIN;
    static const char *SIG_LOGIN_RESP;
    static const char *SIG_LOGOUT;
    static const char *SIG_LOGOUT_RESP;
    static const char *SIG_PING;
    static const char *SIG_PONG;
    static const char *SIG_INFO;
    static const char *SIG_INFO_RESP;

    static const char *SCHEMA_SIG;
    static const char *SCHEMA_SIG_LOGIN;
    static const char *SCHEMA_SIG_LOGIN_RESP;
    static const char *SCHEMA_SIG_LOGOUT;
    static const char *SCHEMA_SIG_LOGOUT_RESP;
    static const char *SCHEMA_SIG_PING;
    static const char *SCHEMA_SIG_PONG;
    static const char *SCHEMA_SIG_INFO;
    static const char *SCHEMA_SIG_INFO_RESP;

    typedef struct FRAME
    {
        unsigned char startFlag;
        unsigned char reserved;
        unsigned char length[2];
        char *payload;

        void setLen(int len)
        {
            assert(!(len & 0xFFFF0000));
            length[0] = len >> 8;
            length[1] = len & 0xFF;
        }
        int len()
        {
            return (length[0] << 8) | length[1];
        }
    } FRAME_t;

    class Initializer
    {
    public:
        Initializer();
    };

    /**
     * parse RCMP signaling from given buffer
     * 
     * pBuf: raw buffer
     * len: in - data lenth; out - parsed frame's size, in byte.
     * 
     * return: corresponding error code
     */
    static int parse(void *pBuf, int &len, rapidjson::Document &doc);

    /**
     * give request cmd and retrive corresponding resp cmd
     * 
     * cmd: request cmd
     * 
     * return: corresponding resp cmd if it has be found; otherwise nullptr 
     */
    static const char *getRespCmd(const char *cmd);

    /**
     * convert given request signaling object to corresponding response signaling.
     * 
     * sig: in - request signaling; out - corresponding response signaling
     * err: error number
     * errmsg: error description
     * payload: payload of response signaling
     * 
     * return: response signaling object(same as request signaling object).
     */
    static rapidjson::Document &convertToResp(rapidjson::Document &sig,
                                              const int err,
                                              const char *errmsg,
                                              rapidjson::Value *payload = nullptr);
    static rapidjson::Document &convertToResp(rapidjson::Document &sig,
                                              rapidjson::Value *payload = nullptr)
    {
        convertToResp(sig, SUCCESS, "", payload);
    }

protected:
    static Initializer gInitializer;
    static std::map<const char *, const char *> gSigRespCmdMap;

    static std::map<const char *, rapidjson::SchemaDocument *> gSchemaMap;
    static std::map<const char *, rapidjson::SchemaValidator *> gValidatorMap;

    static void init();
    static void initSchemaValidator(const char *name, const char *schema);
    static int verifyFrame(FRAME_t *pFrame, int len);
    static bool verifySig(rapidjson::Document &doc);
};

} // namespace roscar_common
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_ROSCARCOMMON_RCMP_H_
