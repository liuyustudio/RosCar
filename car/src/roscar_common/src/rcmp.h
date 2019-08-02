#ifndef _ROSCAR_CAR_ROSCARCOMMON_RCMP_H_
#define _ROSCAR_CAR_ROSCARCOMMON_RCMP_H_

#include <assert.h>
#include <map>

#include "rapidjson/document.h"
#include "rapidjson/schema.h"
#include "rapidjson/stringbuffer.h"

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
    static const char *FIELD_PAYLOAD;
    static const char *FIELD_LOGIN_VER;
    static const char *FIELD_LOGIN_TYPE;
    static const char *FIELD_LOGIN_ID;

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

    RCMP();
    virtual ~RCMP();

    /**
     * parse RCMP signaling from given buffer
     * 
     * pBuf: raw buffer
     * len: in - data lenth; out - parsed frame's size, in byte.
     * 
     * return: corresponding error code
     */
    int parse(void *pBuf, int &len, rapidjson::Document &doc);

    /**
     * convert given signaling object to PONG signaling.
     * 
     * sig: in - original signaling; out - PONG signaling
     * 
     * return: reference of input sig object.
     */
    static rapidjson::Document & convertToPong(rapidjson::Document &sig);

protected:
    std::map<const char *, rapidjson::SchemaDocument *> mSchemaMap;
    std::map<const char *, rapidjson::SchemaValidator *> mValidatorMap;

    void initSchemaValidator(const char * name, const char * schema);
    int verifyFrame(FRAME_t *pFrame, int len);
    bool verifySig(rapidjson::Document &doc);
};

} // namespace roscar_common
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_ROSCARCOMMON_RCMP_H_
