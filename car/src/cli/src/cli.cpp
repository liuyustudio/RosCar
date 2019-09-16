#include "cli.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sstream>
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
    else if (0 == strcasecmp(cmd[0].c_str(), "move"))
    {
        return onCmd_Move(cmd);
    }
    else if (0 == strcasecmp(cmd[0].c_str(), "video"))
    {
        return onCmd_Video(cmd);
    }
    else if ((0 == strcasecmp(cmd[0].c_str(), "q")) ||
             (0 == strcasecmp(cmd[0].c_str(), "quit")))
    {
        ROS_DEBUG("[Cli::onCmd] Cli::onCmdquit");
        return false;
    }
    else
    {
        ROS_ERROR_STREAM("[Cli::onCmd] unsupported command: " << cmd[0]);
        return true;
    }
}

bool Cli::onSignaling(UDSClient::BufType &buffer, Document &sig)
{
    const char *cmd = sig[RCMP::FIELD_CMD].GetString();

    if (strcasecmp(RCMP::SIG_INFO_RESP, cmd) == 0)
    {
        return onSigInfoResp(sig);
    }
    else if (strcasecmp(RCMP::SIG_MOVE_RESP, cmd) == 0)
    {
        return onSigMoveResp(sig);
    }
    else
    {
        ROS_ERROR("[Cli::onSignaling] process routine for signaling[%s] Not implemented yet.", cmd);
        return false;
    }
}

void Cli::sendRequest(string &req)
{
    sendSig(req);
}

bool Cli::onCmd_Info(vector<string> &cmd)
{
    string request =
        R"({
            "cmd": "Info",
            "seq": 0,
            "payload": null
        })";
    request.erase(remove_if(request.begin(), request.end(),
                            [](unsigned char ch) { return isspace(ch); }),
                  request.end());

    // ROS_DEBUG("request: %s", request.c_str());

    // put request into list
    sendRequest(request);

    return true;
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

    ROS_INFO("\tinfo.id: %s", id);
    ROS_INFO("\tinfo.type: %s", type);
    ROS_INFO("\tinfo.name: %s", name);
    printf("> ");
    fflush(nullptr);

    return true;
}

bool Cli::onCmd_Move(vector<string> &cmd)
{
    if (cmd.size() != 4)
    {
        ROS_ERROR("syntax: move [angle] [power] [duration]");
        return true;
    }

    // parse command
    float angle = atof(cmd[1].c_str());
    float power = atof(cmd[2].c_str());
    float duration = atof(cmd[3].c_str());

    stringstream ss;

    ss << R"({"cmd": "Move", "seq": 0, "payload": {)"
       << R"(  "angle":)" << angle
       << R"(, "power": )" << power
       << R"(, "duration": )" << duration
       << R"(}})";

    string &&request = ss.str();

    request.erase(remove_if(request.begin(), request.end(),
                            [](unsigned char ch) { return isspace(ch); }),
                  request.end());

    // put request into list
    sendRequest(request);

    return true;
}

bool Cli::onSigMoveResp(Document &sig)
{
    int _errno = sig[RCMP::FIELD_ERRNO].GetInt();
    if (_errno != SUCCESS)
    {
        // query Move fail
        const char *_errmsg = sig[RCMP::FIELD_ERRMSG].GetString();
        ROS_ERROR("[Cli::onSigMoveResp] move fail. Error: %d. %s", _errno, _errmsg);
        return false;
    }

    ROS_INFO("\t move ok.");
    printf("> ");
    fflush(nullptr);

    return true;
}

bool Cli::onCmd_Video(vector<string> &cmd)
{
    if (cmd.size() < 2)
    {
        ROS_ERROR("syntax: video [list/open] [args...]");
        return true;
    }

    // parse video action
    string request;
    if (0 == strcasecmp(cmd[1].c_str(), "list"))
    {
        // list existed video stream
        request = R"({"cmd": "Video", "seq": 0, "payload": {)"
                  R"(  "action": "list")"
                  R"(}})";
    }
    else if (0 == strcasecmp(cmd[1].c_str(), "open"))
    {
        // open video stream
        if (cmd.size() < 4)
        {
            ROS_ERROR("syntax: video open [nodeId] [videoId]");
            return true;
        }

        stringstream ss;

        ss << R"({"cmd": "Video", "seq": 0, "payload": {)"
           << R"(  "action": "open",)"
           << R"(  "node": ")" << cmd[2] << R"(",)"
           << R"(  "video": ")" << cmd[3] << R"(")"
           << R"(}})";
        request = ss.str();
    }

    // remove space
    request.erase(remove_if(request.begin(), request.end(),
                            [](unsigned char ch) { return isspace(ch); }),
                  request.end());

    // put request into list
    sendRequest(request);

    return true;
}

bool Cli::onSigVideoResp(Document &sig)
{
    int _errno = sig[RCMP::FIELD_ERRNO].GetInt();
    if (_errno != SUCCESS)
    {
        // query Video fail
        const char *_errmsg = sig[RCMP::FIELD_ERRMSG].GetString();
        ROS_ERROR("[Cli::onSigVideoResp] video cmd fail. Error: %d. %s", _errno, _errmsg);
        return false;
    }

    ROS_INFO("\t video cmd ok.");
    printf("> ");
    fflush(nullptr);

    return true;
}

} // namespace cli
} // namespace car
} // namespace roscar
