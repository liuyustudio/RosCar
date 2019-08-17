#include "rcmp.h"

namespace roscar
{
namespace car
{
namespace roscar_common
{

const char *RCMP::SCHEMA_SIG =
    R"({
        "type":"object",
        "properties":{
            "cmd":{
                "type":"string"
            },
            "seq":{
                "type":"integer"
            },
            "payload": {
                "type": ["object", "null"]
            }
        },
        "required":["cmd","seq"]
    })";

const char *RCMP::SCHEMA_SIG_LOGIN =
    R"({
       "type":"object",
       "properties":{
           "cmd":{
               "type":"string"
           },
           "seq":{
               "type":"integer"
           },
           "payload":{
              "type":"object",
              "properties":{
                  "ver":{
                      "type":"integer"
                  },
                  "type":{
                      "type":"string"
                  },
                  "id":{
                      "type":"string"
                  }
              },
              "required":["ver","type","id"]
           }
       },
       "required":["cmd","seq","payload"]
    })";

const char *RCMP::SCHEMA_SIG_LOGIN_RESP =
    R"({
       "type":"object",
       "properties":{
            "cmd":{
                "type":"string"
            },
            "seq":{
                "type":"integer"
            },
            "errno":{
                "type":"integer"
            },
            "errmsg":{
                "type":"string"
            },
            "payload": {
                "type": ["object", "null"]
            }
       },
       "required":["cmd","seq","errno","errmsg","payload"]
    })";

const char *RCMP::SCHEMA_SIG_LOGOUT =
    R"({
       "type":"object",
       "properties":{
            "cmd":{
                "type":"string"
            },
            "seq":{
                "type":"integer"
            },
            "payload": {
                "type": ["object", "null"]
            }
       },
       "required":["cmd","seq"]
    })";

const char *RCMP::SCHEMA_SIG_LOGOUT_RESP = RCMP::SCHEMA_SIG_LOGIN_RESP;

const char *RCMP::SCHEMA_SIG_PING = RCMP::SCHEMA_SIG_LOGOUT;
const char *RCMP::SCHEMA_SIG_PONG = RCMP::SCHEMA_SIG_LOGOUT_RESP;

const char *RCMP::SCHEMA_SIG_INFO = RCMP::SCHEMA_SIG_LOGOUT;
const char *RCMP::SCHEMA_SIG_INFO_RESP =
    R"({
       "type":"object",
       "properties":{
           "cmd":{
               "type":"string"
           },
           "seq":{
               "type":"integer"
           },
           "payload":{
              "type":"object",
              "properties":{
                    "id":{
                        "type":"string"
                    },
                    "type":{
                        "type":"string"
                    },
                    "name":{
                        "type":"string"
                    }
              },
              "required":["id","type","name"]
           }
       },
       "required":["cmd","seq","payload"]
    })";

const char *RCMP::SCHEMA_SIG_MOVE =
    R"({
        "type":"object",
        "properties":{
            "cmd":{
                "type":"string"
            },
            "seq":{
                "type":"integer"
            },
            "payload": {
              "type":"object",
              "properties":{
                    "angle":{
                        "type":"integer"
                    },
                    "power":{
                        "type":"integer"
                    },
                    "duration":{
                        "type":"integer"
                    }
                },
                "required":["angle","power","duration"]
            }
        },
        "required":["cmd","seq","payload"]
    })";

const char *RCMP::SCHEMA_SIG_MOVE_RESP = RCMP::SCHEMA_SIG_LOGIN_RESP;

} // namespace roscar_common
} // namespace car
} // namespace roscar
