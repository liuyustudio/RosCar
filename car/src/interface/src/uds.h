#ifndef _ROSCAR_CAR_INTERFACE_UDS_H_
#define _ROSCAR_CAR_INTERFACE_UDS_H_

#include <string.h>
#include <thread>
#include <vector>
#include "rapidjson/document.h"
#include "roscar_common/rcmp.h"
#include "roscar_common/sessionBuffer.hpp"

namespace roscar
{
namespace car
{
namespace interface
{

class UDS
{
public:
    using BufType = roscar_common::SessionBuffer<>;
    using OnSigCallbak = bool (*)(BufType &, rapidjson::Document &);

    static const int INTERVAL_EPOLL_RETRY;
    static const int INTERVAL_CONNECT_RETRY;

    static const int EPOLL_MAX_EVENTS = 8;

    typedef struct _UdsSession
    {
        int soc;
        uint32_t events;
        BufType buffer;

        inline _UdsSession(int _soc = 0, uint32_t _events = 0) { init(_soc, _events); }
        inline void init(int _soc = 0, uint32_t _events = 0)
        {
            soc = _soc;
            events = _events;
            buffer.init();
        }

        inline bool validate() { return soc && buffer.validate(); }
    } UdsSession_t;

    UDS(OnSigCallbak onSigCallbak);
    virtual ~UDS();

    bool start(const char *udsUri);
    void stop();

    inline void join()
    {
        for (auto &t : mThreadArray)
            t.join();
    }

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
    int mSrvSoc;

    std::vector<std::thread> mThreadArray;
    bool mStopFlag;

    OnSigCallbak mOnSigCallbak;
};

} // namespace interface
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_UDS_H_
