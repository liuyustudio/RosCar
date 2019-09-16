#include "utils.h"
#include <errno.h>
#include <string.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>

using namespace std;

namespace roscar
{
namespace car
{
namespace roscar_common
{

bool Utils::getRouteInfo(list<RouteEntry_t> &entryList)
{
    static const char *PROC_FILE = "/proc/net/route";

    ifstream ifs(PROC_FILE);
    if (!ifs)
    {
        ROS_ERROR("[Utils::getRouteInfo] open proc file(%s) fail: %d - %s",
                  PROC_FILE, errno, strerror(errno));
        return false;
    }

    entryList.clear();

    string line;
    while (getline(ifs, line))
    {
        /**
        output format of /proc/net/route
        Iface  Destination  Gateway   Flags  RefCnt  Use  Metric  Mask      MTU  Window  IRTT                                                       
        eth0   00000000     010011AC  0003   0       0    0       00000000  0    0       0                                                                               
        eth0   000011AC     00000000  0001   0       0    0       0000FFFF  0    0       0
        **/
        istringstream iss(line);
        RouteEntry_t entry;

        if (iss >>
            entry.interface >>
            hex >>
            entry.destination.address >>    // hex
            entry.gateway.address >>        // hex
            entry.flags >>                  // hex
            dec >>
            entry.refCnt >>
            entry.use >>
            entry.metric >>
            hex >>
            entry.mask.address >>           // hex
            dec >>
            entry.mtu >>
            entry.window >>
            entry.irtt)
        {
            entryList.emplace_back(entry);
        }
    }

    return true;
}

bool Utils::getIntfInfo(list<IntfEntry_t> &intfList)
{
    /* Create a socket so we can use ioctl on the file 
     * descriptor to retrieve the interface info. 
     */

    int soc = socket(PF_INET, SOCK_DGRAM, 0);
    if (soc <= 0)
    {
        ROS_ERROR("[Utils::getIntfInfo] create socket fail: %d - %s",
                  errno, strerror(errno));
        return false;
    }

    static const int MAX_INTF_COUNT = 128;

    ifconf cfg;
    ifreq reqs[MAX_INTF_COUNT];
    bool ret = false;

    cfg.ifc_len = sizeof(reqs);
    cfg.ifc_ifcu.ifcu_buf = (caddr_t)reqs;

    if (ioctl(soc, SIOCGIFCONF, &cfg) == 0)
    {
        // int ifc_num, addr, bcast, mask, network, i;
        int count = cfg.ifc_len / sizeof(ifreq);
        for (int i = 0; i < count; ++i)
        {
            // only process AF_INET interface
            if (reqs[i].ifr_addr.sa_family != AF_INET)
            {
                continue;
            }

            IntfEntry_t entry;

            // interface name
            entry.interface = reqs[i].ifr_name;

            // ip addr
            if (ioctl(soc, SIOCGIFADDR, &reqs[i]))
            {
                ROS_ERROR("[Utils::getIntfInfo] query ip addr fail: %d - %s",
                          errno, strerror(errno));
                continue;
            }
            entry.address.address =
                ((sockaddr_in *)(&reqs[i].ifr_addr))->sin_addr.s_addr;

            // netmask
            if (ioctl(soc, SIOCGIFNETMASK, &reqs[i]))
            {
                ROS_ERROR("[Utils::getIntfInfo] query netmask info fail: %d - %s",
                          errno, strerror(errno));
                continue;
            }
            entry.mask.address =
                ((sockaddr_in *)(&reqs[i].ifr_netmask))->sin_addr.s_addr;

            // broadcast
            if (ioctl(soc, SIOCGIFBRDADDR, &reqs[i]))
            {
                ROS_ERROR("[Utils::getIntfInfo] query broadcast info fail: %d - %s",
                          errno, strerror(errno));
                continue;
            }
            entry.broadcast.address =
                ((sockaddr_in *)(&reqs[i].ifr_broadaddr))->sin_addr.s_addr;

            intfList.emplace_back(entry);

            ret = true;
        }
    }
    else
    {
        ROS_ERROR("[Utils::getIntfInfo] query intf info fail: %d - %s",
                  errno, strerror(errno));
    }

    close(soc);

    return ret;
}

bool Utils::getDefaultIp(IPv4_t & ip) {
    // get interfaces info
    list<RouteEntry_t> entryList;
    if (!getRouteInfo(entryList)) {
        ROS_ERROR("[Utils::getDefaultIp] get interfaces info fail.");
        return false;
    }

    // get routes info
    list<IntfEntry_t> intfList;
    if (!getIntfInfo(intfList)) {
        ROS_ERROR("[Utils::getDefaultIp] get routes info fail.");
        return false;
    }

    // searching ip to default router
    for (auto &routeEntry: entryList) {
        if (routeEntry.destination.address == 0) {
            for (auto & intfEntry: intfList) {
                if (routeEntry.interface.compare(intfEntry.interface) == 0) {
                    ip = intfEntry.address;
                    ROS_DEBUG_STREAM("[Utils::getDefaultIp] default IP" << ip);
                    return true;
                }
            }
        }
    }

    return false;
}

} // namespace roscar_common
} // namespace car
} // namespace roscar
