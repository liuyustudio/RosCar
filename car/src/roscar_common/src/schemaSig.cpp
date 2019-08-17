namespace roscar
{
namespace car
{
namespace roscar_common
{

const char *RCMP::SCHEMA_SIG =
    R"({)"
    R"(   \"type\":\"object\",)"
    R"(   \"properties\":{)"
    R"(       \"cmd\":{)"
    R"(           \"type\":\"string\")"
    R"(       },)"
    R"(       \"seq\":{)"
    R"(           \"type\":\"integer\")"
    R"(       })"
    R"(   },)"
    R"(   \"required\":[\"cmd\",\"seq\"])"
    R"(})";

const char *RCMP::SCHEMA_SIG_LOGIN =
    R"({)"
    R"(   \"type\":\"object\",)"
    R"(   \"properties\":{)"
    R"(       \"cmd\":{)"
    R"(           \"type\":\"string\")"
    R"(       },)"
    R"(       \"seq\":{)"
    R"(           \"type\":\"integer\")"
    R"(       },)"
    R"(       \"payload\":{)"
    R"(          \"type\":\"object\",)"
    R"(          \"properties\":{)"
    R"(              \"ver\":{)"
    R"(                  \"type\":\"integer\")"
    R"(              },)"
    R"(              \"type\":{)"
    R"(                  \"type\":\"string\")"
    R"(              },)"
    R"(              \"id\":{)"
    R"(                  \"type\":\"string\")"
    R"(              })"
    R"(          },)"
    R"(          \"required\":[\"ver\",\"type\",\"id\"])"
    R"(       })"
    R"(   },)"
    R"(   \"required\":[\"cmd\",\"seq\",\"payload\"])"
    R"(})";

const char *RCMP::SCHEMA_SIG_LOGIN_RESP =
    R"({)"
    R"(   \"type\":\"object\",)"
    R"(   \"properties\":{)"
    R"(       \"cmd\":{)"
    R"(           \"type\":\"string\")"
    R"(       },)"
    R"(       \"seq\":{)"
    R"(           \"type\":\"integer\")"
    R"(       },)"
    R"(       \"errno\":{)"
    R"(           \"type\":\"integer\")"
    R"(       },)"
    R"(       \"errmsg\":{)"
    R"(           \"type\":\"string\")"
    R"(       },)"
    R"(       \"payload\":{)"
    R"(           \"type\":\"object\")"
    R"(       })"
    R"(   },)"
    R"(   \"required\":[\"cmd\",\"seq\",\"errno\",\"errmsg\",\"payload\"])"
    R"(})";

const char *RCMP::SCHEMA_SIG_LOGOUT =
    R"({)"
    R"(   \"type\":\"object\",)"
    R"(   \"properties\":{)"
    R"(       \"cmd\":{)"
    R"(           \"type\":\"string\")"
    R"(       },)"
    R"(       \"seq\":{)"
    R"(           \"type\":\"integer\")"
    R"(       })"
    R"(   },)"
    R"(   \"required\":[\"cmd\",\"seq\"])"
    R"(})";

const char *RCMP::SCHEMA_SIG_LOGOUT_RESP = RCMP::SCHEMA_SIG_LOGIN_RESP;

const char *RCMP::SCHEMA_SIG_PING = RCMP::SCHEMA_SIG_LOGOUT;
const char *RCMP::SCHEMA_SIG_PONG = RCMP::SCHEMA_SIG_LOGOUT_RESP;

const char *RCMP::SCHEMA_SIG_INFO = RCMP::SCHEMA_SIG_LOGOUT;
const char *RCMP::SCHEMA_SIG_INFO_RESP =
    R"({)"
    R"(   "type":"object",)"
    R"(   "properties":{)"
    R"(       "cmd":{)"
    R"(           "type":"string")"
    R"(       },)"
    R"(       "seq":{)"
    R"(           "type":"integer")"
    R"(       },)"
    R"(       "payload":{)"
    R"(          "type":"object",)"
    R"(          "properties":{)"
    R"(              "id":{)"
    R"(                  "type":"string")"
    R"(              },)"
    R"(              "type":{)"
    R"(                  "type":"string")"
    R"(              },)"
    R"(              "name":{)"
    R"(                  "type":"string")"
    R"(              })"
    R"(          },)"
    R"(          "required":["id","type","name"])"
    R"(       })"
    R"(   },)"
    R"(   "required":["cmd","seq","payload"])"
    R"(})";

const char *RCMP::SCHEMA_SIG_MOVE =
    R"({)"
    R"(   "type":"object",)"
    R"(   "properties":{)"
    R"(       "cmd":{)"
    R"(           "type":"string")"
    R"(       },)"
    R"(       "seq":{)"
    R"(           "type":"integer")"
    R"(       })"
    R"(   },)"
    R"(   "required":["cmd","seq"])"
    R"(})";

const char *RCMP::SCHEMA_SIG_MOVE_RESP = RCMP::SCHEMA_SIG_LOGIN_RESP;

} // namespace roscar_common
} // namespace car
} // namespace roscar
