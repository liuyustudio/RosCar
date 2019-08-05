#include "cli.h"

#include <string.h>

#include "roscar_common/error.h"

using namespace std;

namespace roscar
{
namespace car
{
namespace cli
{

const std::chrono::milliseconds Cli::THREAD_WAIT_TIMEOUT(100 * 1000);

Cli::Cli() : mStop(false)
{
}

Cli::~Cli()
{
    // mThread.join();
}

bool Cli::init(const char *udsUri)
{
    /**
     * TODO:
     *  1. uds thread
     */
}

bool Cli::onCmd(std::vector<std::string> &cmd)
{
    if (strcasecmp(cmd[0].c_str(), "info") == 0)
    {
        return onCmd_Info(cmd);
    }
}

void Cli::threadFunc()
{
    unique_lock<mutex> lck(mCvMutex);
    while (!mStop)
    {
        mCv.wait_for(lck, THREAD_WAIT_TIMEOUT);

        if (mReqList.empty())
        {
            // timeout
            continue;
        }

        // TODO: send request
    }
}

void Cli::sendRequest(std::string &req)
{
    unique_lock<mutex> lck(mCvMutex);

    lck.lock();

    mReqList.push_back(req);

    lck.unlock();

    mCv.notify_one();
}

bool Cli::onCmd_Info(std::vector<std::string> &cmd)
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
