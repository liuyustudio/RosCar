#include "cli.h"

#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "roscar_common/error.h"

using namespace std;

namespace roscar
{
namespace car
{
namespace cli
{

const char *Cli::UNIX_DOMAIN_SOCKET_URI = "/tmp/.roscar.car.interface.soc";

Cli::Cli()
    : mStop(false), mUdsFd(0)
{
    // init
    init();
}

Cli::~Cli()
{
    // waiting for thread stop
    for (auto &thread : mThreadArray)
    {
        thread.join();
    }

    // close Unix Domain Socket
    if (mUdsFd)
    {
        close(mUdsFd);
        mUdsFd = 0;
    }
}

void Cli::init()
{
    // init Unix Domain Socket
    {
        // create socket
        if ((mUdsFd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
        {
            throw("create UDS socket fail.");
        }

        // connect
        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, UNIX_DOMAIN_SOCKET_URI, sizeof(addr.sun_path) - 1);
        if (connect(mUdsFd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        {
            close(mUdsFd);
            mUdsFd = 0;
            throw("connect to UDS server fail.");
        }
    }

    // start UDS thread
    mThreadArray.emplace_back(&Cli::udsThreadFunc, this);
}

bool Cli::onCmd(vector<string> &cmd)
{
    if (strcasecmp(cmd[0].c_str(), "info") == 0)
    {
        return onCmd_Info(cmd);
    }
}

void Cli::udsThreadFunc()
{
    unique_lock<mutex> lck(mCvMutex);
    while (!mStop)
    {
        mCv.wait(lck, [=] { return mStop || !mReqList.empty(); });

        if (mReqList.empty())
        {
            // timeout
            continue;
        }

        // TODO: send request
    }
}

void Cli::sendRequest(string &req)
{
    {
        unique_lock<mutex> lck(mCvMutex);
        mReqList.push_back(req);
    }

    mCv.notify_one();
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

} // namespace cli
} // namespace car
} // namespace roscar
