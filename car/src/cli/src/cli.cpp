#include "cli.h"

#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>

#include "roscar_common/error.h"
#include "roscar_common/rcmp.h"

using namespace std;
using namespace rapidjson;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace cli
{

bool Cli::onCmd(vector<string> &cmd)
{
    if (0 == strcasecmp(cmd[0].c_str(), "info"))
    {
        return onCmd_Info(cmd);
    }
    else if ((0 == strcasecmp(cmd[0].c_str(), "q")) ||
             (0 == strcasecmp(cmd[0].c_str(), "quit")))
    {
        ROS_DEBUG("[Cli::onCmd] Cli::onCmdquit");
        return false;
    }
    else
    {
        ROS_DEBUG_STREAM("[Cli::onCmd] unsupported command: " << cmd[0]);
        return true;
    }
}

void Cli::sendRequest(string &req)
{
    sendSig(req);
}

bool Cli::onCmd_Info(vector<string> &cmd)
{
    string INFO_REQUEST = "{"
                          "  \"cmd\": \"Info\","
                          "  \"seq\": 0,"
                          "  \"payload\": null"
                          "}";

    // put request into list
    sendRequest(INFO_REQUEST);

    return true;
}

bool Cli::onSignaling(UDSClient::SESSION_t &sess, Document &sig)
{
    const char *cmd = sig[RCMP::FIELD_CMD].GetString();

    if (strcasecmp(RCMP::SIG_INFO_RESP, cmd) == 0)
    {
        return onSigInfoResp(sig);
    }
    else
    {
        ROS_ERROR("[Cli::onSignaling] process routine for signaling[%s] Not implemented yet.", cmd);
        return false;
    }
}

bool Cli::onSigInfoResp(Document &sig)
{
    int _errno = sig[RCMP::FIELD_ERRNO].GetInt();
    if (_errno != SUCCESS)
    {
        // query Info fail
        const char *_errmsg = sig[RCMP::FIELD_ERRMSG].GetString();
        ROS_ERROR("[Cli::onSigInfoResp] query info fail. Error: %d. %s", _errno, _errmsg);
        return false;
    }

    Value &&payload = sig[RCMP::FIELD_PAYLOAD].GetObject();
    const char *id = payload[RCMP::FIELD_INFORESP_ID].GetString();
    const char *type = payload[RCMP::FIELD_INFORESP_TYPE].GetString();
    const char *name = payload[RCMP::FIELD_INFORESP_NAME].GetString();

    printf("\tinfo.id: %s\n", id);
    printf("\tinfo.type: %s\n", type);
    printf("\tinfo.name: %s\n", name);

    return true;
}

} // namespace cli
} // namespace car
} // namespace roscar
