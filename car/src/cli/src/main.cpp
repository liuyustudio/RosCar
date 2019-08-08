
#include <stdio.h>
#include <stdlib.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include "cli.h"
#include "udsClient.h"

using namespace std;
using namespace roscar::car::cli;

static const char *DELIM = " ";
static const char *UNIX_DOMAIN_SOCKET_URI = "/tmp/.roscar.car.interface.soc";

bool onSignaling(UDSClient::SESSION_t &sess, rapidjson::Document &sig);
bool sendSignaling(std::string &sig);
bool process(const char *line);

UDSClient udsClient(onSignaling);
Cli cli(sendSignaling);

int main(int argc, char **argv)
{
    ROS_INFO("\nROS Car CLI\n\n");

    char *line = nullptr;
    bool ret = true;

    // init udsClient
    if (!udsClient.start(UNIX_DOMAIN_SOCKET_URI))
    {
        ROS_ERROR("Err: init udsClient fail.");
        return EXIT_FAILURE;
    }

    while (true)
    {
        if (!(line = readline("> ")))
        {
            // EOF
            break;
        }

        if (*line)
        {
            add_history(line);
            ret = process(line);
        }

        free(line);
        if (!ret)
        {
            // routine finished
            break;
        }
    }

    ROS_INFO("\nROS Car CLI: Bye\n\n");
}

bool onSignaling(UDSClient::SESSION_t &sess, rapidjson::Document &sig)
{
    return cli.onSignaling(sess, sig);
}

bool sendSignaling(std::string &sig)
{
    return udsClient.sendRawString(sig);
}

bool process(const char *line)
{
    vector<string> cmd;
    stringstream ss(line);
    string item;

    while (ss >> item)
    {
        cmd.push_back(item);
    }

    return cli.onCmd(cmd);
}
