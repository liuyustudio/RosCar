#ifndef _ROSCAR_CAR_CLI_CLI
#define _ROSCAR_CAR_CLI_CLI

#include <list>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace roscar
{
namespace car
{
namespace cli
{

class Cli
{
    static const char * UNIX_DOMAIN_SOCKET_URI;

public:
    Cli();
    virtual ~Cli();

    /**
     * @brief init Cli object
     */
    void init();

    /**
     * @brief process new input command
     * 
     * @param cmd: input command
     * @return true: waiting for next command.
     * @return false: stop process.
     */
    bool onCmd(std::vector<std::string> &cmd);

protected:
    std::mutex mCvMutex;
    std::condition_variable mCv;
    std::list<std::string> mReqList;
    std::vector<std::thread> mThreadArray;
    bool mStop;
    int mUdsFd;

    void udsThreadFunc();

    void sendRequest(std::string &req);
    bool onCmd_Info(std::vector<std::string> &cmd);
};

} // namespace cli
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_CLI_CLI
