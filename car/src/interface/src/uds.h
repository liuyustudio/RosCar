#ifndef _ROSCAR_CAR_INTERFACE_UDS_H_
#define _ROSCAR_CAR_INTERFACE_UDS_H_

#include <thread>
#include <vector>
#include "rapidjson/document.h"
#include "roscar_common/rcmp.h"

namespace roscar
{
namespace car
{
namespace interface
{

class UDS
{
public:
    static const int EPOLL_TIMEOUT = 1 * 1000;
    static const int EPOLL_RETRY_INTERVAL = 1 * 100 * 1000;
    static const int EPOLL_MAX_EVENTS = 16;
    static const int MAX_CLIENT_COUNT = 8;
    static const int RECV_BUFFER_CAPACITY = 64 * 1024;
    static const int SEND_BUFFER_CAPACITY = 64 * 1024;

    typedef struct SESSION
    {
        int soc;
        unsigned int events;
        unsigned int recvBufPos = 0;
        unsigned int recvBufEnd = 0;
        unsigned int sendBufPos = 0;
        unsigned int sendBufEnd = 0;
        char recvBuf[RECV_BUFFER_CAPACITY];
        char sendBuf[SEND_BUFFER_CAPACITY];

        SESSION() { init(); }
        inline void init() { soc = events = recvBufPos = recvBufEnd = sendBufPos = sendBufEnd = 0; }
        inline bool isSendBufEmpty() { return sendBufPos == sendBufEnd; }
    } SESSION_t;

    typedef bool (*FUNC_ONSIGLAING)(UDS::SESSION_t &sess, rapidjson::Document &sig);

    UDS(FUNC_ONSIGLAING cb_onSig);
    virtual ~UDS();

    bool start(const char *udsUri);
    void stop();

    inline void join()
    {
        for (auto &t : mThreadArray)
        {
            t.join();
        }
    }

protected:
    void threadFunc();

    bool onSession(unsigned int socEvents, SESSION_t &sess);
    bool onRead(SESSION_t &sess);
    bool onWrite(SESSION_t &sess);
    int parseSig(SESSION_t &sess, rapidjson::Document &doc);

    bool mStopUDS;
    int mEpollfd = 0;
    int mUdsSoc = 0;

    std::vector<std::thread> mThreadArray;

    FUNC_ONSIGLAING mCb_OnSig;
};

} // namespace interface
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_UDS_H_
