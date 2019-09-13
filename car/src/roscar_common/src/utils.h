#ifndef _ROSCAR_CAR_ROSCARCOMMON_UTILS_H_
#define _ROSCAR_CAR_ROSCARCOMMON_UTILS_H_

#include <string>
#include <list>
#include "define.h"

namespace roscar
{
namespace car
{
namespace roscar_common
{

class Utils
{
protected:
    Utils() = default;
    Utils(const Utils &src) = default;
    Utils &operator=(const Utils &src) { return *this; }

public:
    // query IPv4 route table
    static bool getRouteInfo(std::list<RouteEntry_t> &entryList);
    static bool getIntfInfo(std::list<IntfEntry_t> &intfList);
    static bool getDefaultIp(IPv4_t & ip);
};

} // namespace roscar_common
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_ROSCARCOMMON_UTILS_H_
