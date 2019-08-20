#ifndef _ROSCAR_CAR_CLI_UDS_H_
#define _ROSCAR_CAR_CLI_UDS_H_

#include <string.h>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include "rapidjson/document.h"
#include "roscar_common/rcmp.h"
#include "roscar_common/sessionBuffer.hpp"

namespace roscar
{
namespace car
{
namespace cli
{

class UDSClient
{
public:
    using BufType = roscar_common::SessionBuffer<>;
    using OnSigCallbak = bool (*)(BufType &buffer, rapidjson::Document &sig);

    static const int INTERVAL_EPOLL_RETRY;
    static const int INTERVAL_CONNECT_RETRY;

    static const int EPOLL_MAX_EVENTS = 16;
    static const int MAX_CLIENT_COUNT = 8;

    typedef struct _UdsSession
    {
        int soc;
        uint32_t events;
        BufType buffer;

        inline _UdsSession() { init(); }
        inline void init()
        {
            soc = 0;
            events = 0;
            buffer.init();
        }

        inline bool validate() { return soc && buffer.validate(); }
    } UdsSession_t;

    UDSClient(OnSigCallbak onSigCallbak);
    virtual ~UDSClient();

    bool start(const char *udsUri);
    void stop();

    bool sendSig(std::string &req);
    inline bool sendSig(std::string &&req) { return sendSig(req); }
    inline bool sendSig(rapidjson::Document &req) { return sendSig(roscar_common::RCMP::getJson(req)); }

protected:
    void threadFunc();
    bool initEnv();
    void closeEnv();

    bool onSoc(unsigned int socEvents, UdsSession_t &sess);
    bool onRead(UdsSession_t &sess);
    bool onWrite(UdsSession_t &sess);
    bool setSocWritable(UdsSession_t &sess, bool writable);

    int parseSig(BufType &buffer, rapidjson::Document &sig);

    std::string mUdsUri;
    int mEpollfd;
    UdsSession_t mUdsSession;

    std::vector<std::thread> mThreadArray;
    bool mStopFlag;

    std::mutex mAccessMutex;

    OnSigCallbak mOnSigCallbak;
};

} // namespace cli
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_CLI_UDS_H_
