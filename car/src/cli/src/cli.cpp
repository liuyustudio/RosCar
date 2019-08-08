#include "cli.h"

#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>

#include "roscar_common/error.h"

using namespace std;

namespace roscar
{
namespace car
{
namespace cli
{

bool Cli::onCmd(vector<string> &cmd)
{
    if (strcasecmp(cmd[0].c_str(), "info") == 0)
    {
        return onCmd_Info(cmd);
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

bool Cli::onSignaling(UDSClient::SESSION_t &sess, rapidjson::Document &sig)
{
    // TODO: ...
    ROS_ERROR("Err: Not implemented yet.");
    return false;
}

} // namespace cli
} // namespace car
} // namespace roscar
