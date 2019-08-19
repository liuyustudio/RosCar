#ifndef _ROSCAR_CAR_ROSCARCOMMON_SESSION_HPP_
#define _ROSCAR_CAR_ROSCARCOMMON_SESSION_HPP_

#include <string.h>
#include <tuple>

namespace roscar
{
namespace car
{
namespace roscar_common
{

static const int DEFAULT_SESSION_BUFFER_SIZE = 1 * 1024;

template <int RECV_CAPACITY = DEFAULT_SESSION_BUFFER_SIZE, int SEND_CAPACITY = DEFAULT_SESSION_BUFFER_SIZE>
struct SessionBuffer
{
    using BufInfo = std::tuple<char *, unsigned int>;

    int soc;
    unsigned int events = 0;

    unsigned int recvBufStart = 0;
    unsigned int recvBufEnd = 0;
    unsigned int sendBufStart = 0;
    unsigned int sendBufEnd = 0;

    char recvBuf[RECV_CAPACITY];
    char sendBuf[SEND_CAPACITY];

    inline bool isRecvBufEmpty() { return recvBufStart == recvBufEnd; }
    inline bool isRecvBufFull() { return RECV_CAPACITY == recvBufEnd ? defragRecvBuf() : true; }
    inline bool isSendBufEmpty() { return sendBufStart == sendBufEnd; }
    inline bool isSendBufFull() { return SEND_CAPACITY == sendBufEnd ? defragSendBuf() : true; }
    inline bool defragRecvBuf()
    {
        if (0 == recvBufStart)
        {
            return RECV_CAPACITY == recvBufEnd ? false : true;
        }

        memmove(recvBuf, recvBuf + recvBufStart, recvBufStart);
        recvBufEnd -= recvBufStart;
        recvBufStart = 0;
        return true;
    }
    inline bool defragSendBuf()
    {
        if (0 == sendBufStart)
        {
            return SEND_CAPACITY == sendBufEnd ? false : true;
        }

        memmove(sendBuf, sendBuf + sendBufStart, sendBufStart);
        sendBufEnd -= sendBufStart;
        sendBufStart = 0;
        return true;
    }
    inline bool defrag() { return defragRecvBuf() && defragSendBuf(); }

    inline unsigned int getRecvBufSize() { return isRecvBufFull() ? 0 : RECV_CAPACITY - recvBufEnd; }
    inline unsigned int getSendBufSize() { return isSendBufFull() ? 0 : SEND_CAPACITY - sendBufEnd; }
    inline unsigned int getRecvDataSize() { return recvBufEnd - recvBufStart; }
    inline unsigned int getSendDataSize() { return recvBufEnd - recvBufStart; }

    BufInfo getRecvBuf()
    {
        unsigned int size = getRecvBufSize();
        return 0 < size ? BufInfo(recvBuf + recvBufEnd, size) : BufInfo(nullptr, 0);
    }
    BufInfo getSendBuf()
    {
        unsigned int size = getSendBufSize();
        return 0 < size ? BufInfo(sendBuf + sendBufEnd, size) : BufInfo(nullptr, 0);
    }

    BufInfo getRecvData()
    {
        unsigned int size = getRecvDataSize();
        return 0 < size ? BufInfo(recvBuf + recvBufStart, size) : BufInfo(nullptr, 0);
    }
    BufInfo getSendData()
    {
        unsigned int size = getSendDataSize();
        return 0 < size ? BufInfo(sendBuf + sendBufStart, size) : BufInfo(nullptr, 0);
    }
};

} // namespace roscar_common
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_ROSCARCOMMON_SESSION_HPP_
