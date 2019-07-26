#ifndef _ROSCAR_CAR_INTERFACE_RCMP_H_
#define _ROSCAR_CAR_INTERFACE_RCMP_H_

namespace roscar
{
namespace car
{
namespace interface
{

class RCMP
{
public:
    static const int RCMP_FRAMEHEADSIZE = 4;
    static const char RCMP_STARTFLAG = 0x5A;
    static const int RCMP_VERSION = 1;
    static const int RCMP_MIN_FRAME_SIZE = RCMP_FRAMEHEADSIZE;
    static const int RCMP_MAXPAYLOAD = 0x10000 - RCMP_FRAMEHEADSIZE;

    typedef struct FRAME
    {
        unsigned char startFlag;
        unsigned char reserved;
        unsigned short length;
        char * payload;
    } FRAME_t;

};

} // namespace interface
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_RCMP_H_
