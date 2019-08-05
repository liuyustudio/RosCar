#ifndef ROSCAR_CAR_CLI_CLI
#define ROSCAR_CAR_CLI_CLI

#include <list>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
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
    static const std::chrono::milliseconds THREAD_WAIT_TIMEOUT;

public:
    Cli();
    virtual ~Cli();

    /**
     * @brief init Cli object
     * 
     * @param udsUri: URI of Unix Domain Socket
     * @return true: if init ok
     * @return false: if init fail
     */
    bool init(const char *udsUri);

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

    // std::thread mThread;
    bool mStop;

    void threadFunc();

    void sendRequest(std::string &req);
    bool onCmd_Info(std::vector<std::string> &cmd);
};

} // namespace cli
} // namespace car
} // namespace roscar

#endif // ROSCAR_CAR_CLI_CLI
