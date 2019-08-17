#ifndef _ROSCAR_CAR_INTERFACE_UDS_H_
#define _ROSCAR_CAR_INTERFACE_UDS_H_

#include <string.h>
#include <mutex>
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
    static const int INTERVAL_EPOLL_RETRY;
    static const int INTERVAL_CONNECT_RETRY;

    static const int EPOLL_MAX_EVENTS = 16;
    static const int MAX_CLIENT_COUNT = 8;
    static const int RECV_BUFFER_CAPACITY = 8 * 1024;
    static const int SEND_BUFFER_CAPACITY = 8 * 1024;

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
        inline bool isRecvBufEmpty() { return recvBufPos == recvBufEnd; }
        inline bool isRecvBufFull() { return RECV_BUFFER_CAPACITY == recvBufEnd; }
        inline bool isSendBufEmpty() { return sendBufPos == sendBufEnd; }
        inline bool isSendBufFull() { return SEND_BUFFER_CAPACITY == sendBufEnd; }
        inline bool defragRecvBuf()
        {
            if (0 == recvBufPos)
            {
                return isSendBufFull() ? false : true;
            }

            memmove(recvBuf, recvBuf + recvBufPos, recvBufPos);
            recvBufEnd -= recvBufPos;
            recvBufPos = 0;
            return true;
        }
        inline bool defragSendBuf()
        {
            if (0 == sendBufPos)
            {
                return isSendBufFull() ? false : true;
            }

            memmove(sendBuf, sendBuf + sendBufPos, sendBufPos);
            sendBufEnd -= sendBufPos;
            sendBufPos = 0;
            return true;
        }
        inline bool defrag() { return defragRecvBuf() && defragSendBuf(); }
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
    bool initEnv(int & epollfd, int & soc);
    void closeEnv(int & epollfd, int & soc);

    bool onSession(int epollfd, unsigned int socEvents, SESSION_t &sess);
    bool onRead(int epollfd, SESSION_t &sess);
    bool onWrite(int epollfd, SESSION_t &sess);
    int parseSig(SESSION_t &sess, rapidjson::Document &doc);

    std::string mUdsUri;
    bool mStopFlag;

    std::mutex mAccessMutex;
    std::vector<std::thread> mThreadArray;

    FUNC_ONSIGLAING mCb_OnSig;
};

} // namespace interface
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_UDS_H_
