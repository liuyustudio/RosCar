#ifndef _ROSCAR_CAR_ROSCARCOMMON_SCHEMASIG_H_
#define _ROSCAR_CAR_ROSCARCOMMON_SCHEMASIG_H_

namespace roscar
{
namespace car
{
namespace roscar_common
{

// request & response
const char *SCHEMA_SIG_REQ =
    R"({
        "$schema": "http://json-schema.org/draft-07/schema#",
        "type": "object",
        "properties": {
            "cmd": { "type": "string" },
            "seq": { "type": "integer" },
            "payload": { "type": ["object", "null"] }
        },
        "required":["cmd","seq","payload"]
    })";
const char *SCHEMA_SIG_RESP =
    R"({
        "$schema": "http://json-schema.org/draft-07/schema#",
        "type": "object",
        "properties": {
            "cmd": { "type": "string" },
            "seq": { "type": "integer" },
            "errno": { "type": "integer" },
            "errmsg": { "type": "string" },
            "payload": { "type": ["object", "null"] }
        },
        "required":["cmd","seq","errno","errmsg","payload"]
    })";

// signaling
const char *SCHEMA_SIG = SCHEMA_SIG_REQ;

// login
const char *SCHEMA_SIG_LOGIN =
    R"({
        "$schema": "http://json-schema.org/draft-07/schema#",
        "type": "object",
        "properties": {
            "cmd": { "type": "string" },
            "seq": { "type": "integer" },
            "payload": {
                "type": "object",
                "properties": {
                    "ver": { "type": "integer" },
                    "type": { "type": "string" },
                    "id": { "type": "string" }
                },
                "required":["ver","type","id"]
            }
        },
        "required":["cmd","seq","payload"]
    })";
const char *SCHEMA_SIG_LOGIN_RESP = SCHEMA_SIG_RESP;

// logout
const char *SCHEMA_SIG_LOGOUT = SCHEMA_SIG_REQ;
const char *SCHEMA_SIG_LOGOUT_RESP = SCHEMA_SIG_RESP;

// ping & pong
const char *SCHEMA_SIG_PING = SCHEMA_SIG_REQ;
const char *SCHEMA_SIG_PONG = SCHEMA_SIG_RESP;

// info
const char *SCHEMA_SIG_INFO = SCHEMA_SIG_REQ;
const char *SCHEMA_SIG_INFO_RESP =
    R"({
        "$schema": "http://json-schema.org/draft-07/schema#",
        "type": "object",
        "properties": {
            "cmd": { "type": "string" },
            "seq": { "type": "integer" },
            "payload": {
                "type": "object",
                "properties": {
                    "id": { "type": "string" },
                    "type": { "type": "string" },
                    "name": { "type": "string" }
                },
                "required":["id","type","name"]
            }
        },
        "required":["cmd","seq","payload"]
    })";

// move
const char *SCHEMA_SIG_MOVE =
    R"({
        "$schema": "http://json-schema.org/draft-07/schema#",
        "type": "object",
        "properties": {
            "cmd": { "type": "string" },
            "seq": { "type": "integer" },
            "payload": {
                "type": "object",
                "properties": {
                    "angle": { "type": "integer" },
                    "power": { "type": "integer" },
                    "duration": { "type": "integer" }
                },
                "required":["angle","power","duration"]
            }
        },
        "required":["cmd","seq","payload"]
    })";
const char *SCHEMA_SIG_MOVE_RESP = SCHEMA_SIG_RESP;

// video
const char *SCHEMA_SIG_VIDEO =
    R"({
        "$schema": "http://json-schema.org/draft-07/schema#",
        "type": "object",
        "properties": {
            "cmd": { "type": "string" },
            "seq": { "type": "integer" },
            "payload": {
                "type": "object",
                "anyOf": [
                    {
                        "properties": {
                            "action": { "const": "list" },
                        }
                    },
                    {
                        "properties": {
                            "action": { "const": "open" },
                            "node": { "type": "string" },
                            "video": { "type": "string" }
                        },
                        "required": ["node", "video"]
                    }
                ]
            }
        },
        "required":["cmd","seq","payload"]
    })";

const char *SCHEMA_SIG_VIDEO_RESP = SCHEMA_SIG_RESP;

} // namespace roscar_common
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_ROSCARCOMMON_SCHEMASIG_H_
