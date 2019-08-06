#ifndef _ROSCAR_CAR_CLI_UDS_H_
#define _ROSCAR_CAR_CLI_UDS_H_

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
    static const int EPOLL_TIMEOUT = 1 * 1000;
    static const int EPOLL_RETRY_INTERVAL = 1 * 100 * 1000;
    static const int EPOLL_MAX_EVENTS = 16;
    static const int MAX_CLIENT_COUNT = 8;
    static const int RECV_BUFFER_CAPACITY = 64 * 1024;
    static const int SEND_BUFFER_CAPACITY = 64 * 1024;
    static const char *UDS_PATH;

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
    } SESSION_t;

    typedef bool (*FUNC_ONSIGLAING)(UDSClient::SESSION_t &sess, rapidjson::Document &sig);

    UDSClient(FUNC_ONSIGLAING cb_onSig);
    virtual ~UDSClient();

    void sendRawString(std::string &reqString);
    void sendSig(rapidjson::Document &sig);

    void threadFunc();

protected:
    bool onSoc(SESSION_t &sess);
    bool onRead(SESSION_t &sess);
    bool onWrite(SESSION_t &sess);
    bool parseSig(SESSION_t &sess, rapidjson::Document &doc);

    bool init();
    void teardown();

    bool mStopUDS;
    int mEpollfd = 0;
    int mUdsSoc = 0;

    FUNC_ONSIGLAING mCb_OnSig;
};

} // namespace cli
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_CLI_UDS_H_
