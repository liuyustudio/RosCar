#include "videoTopic.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "ros/ros.h"

#include "roscar_common/error.h"

using namespace std;
using namespace roscar::car::roscar_common;

namespace roscar
{
namespace car
{
namespace videonode
{

void VideoTopic::initSession(VideoTopic::Session_t &session)
{
    static const char *BASE_TABLE = "0123456789ABCDEF";

    // assign random topic name
    srand(time(nullptr));
    char *p = session.name;
    for (int i = 0; i < 2; ++i)
    {
        int r = rand();
        for (int j = 0; j < 4; ++j, ++p, r >>= 4)
        {
            *p = BASE_TABLE[r & 0x0F];
        }
    }
}

} // namespace videonode
} // namespace car
} // namespace roscar
