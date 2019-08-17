#ifndef _ROSCAR_CAR_CLI_UDS_H_
#define _ROSCAR_CAR_CLI_UDS_H_

#include <string.h>
#include <mutex>
#include <thread>
#include <string>
#include <list>
#include <vector>
#include "rapidjson/document.h"
#include "roscar_common/rcmp.h"

namespace roscar
{
namespace car
{
namespace cli
{

class UDSClient
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
        unsigned int events = 0;
        unsigned int recvBufPos = 0;
        unsigned int recvBufEnd = 0;
        unsigned int sendBufPos = 0;
        unsigned int sendBufEnd = 0;
        char recvBuf[RECV_BUFFER_CAPACITY];
        char sendBuf[SEND_BUFFER_CAPACITY];

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

    typedef bool (*FUNC_ONSIGLAING)(UDSClient::SESSION_t &sess, rapidjson::Document &sig);

    UDSClient(FUNC_ONSIGLAING cb_onSig);
    virtual ~UDSClient();

    bool start(const char *udsUri);
    void stop();

    bool sendRawString(std::string &req);
    bool sendSig(rapidjson::Document &req);

protected:
    void threadFunc();
    bool initEnv(int & epollfd, int & soc);
    void closeEnv(int & epollfd, int & soc);

    bool onSoc(int epollfd, unsigned int socEvents, SESSION_t &sess);
    bool onRead(int epollfd, SESSION_t &sess);
    bool onWrite(int epollfd, SESSION_t &sess);
    int parseRawBuffer(SESSION_t &sess, rapidjson::Document &sig);
    bool sendToBuf(SESSION_t &sess, rapidjson::Document &sig);
    bool sendToBuf(SESSION_t &sess, std::string &sig);
    bool setWriteFlag(int epollfd, SESSION_t &sess, bool writeFlag = true);

    std::string mUdsUri;
    std::vector<std::thread> mThreadArray;
    bool mStopFlag;

    std::mutex mAccessMutex;
    std::list<std::string> mReqStrList;

    FUNC_ONSIGLAING mCb_OnSig;
};

} // namespace cli
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_CLI_UDS_H_
