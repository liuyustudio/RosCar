#ifndef _ROSCAR_CAR_ROSCARCOMMON_DEFAULT_H_
#define _ROSCAR_CAR_ROSCARCOMMON_DEFAULT_H_

#include <arpa/inet.h>
#include <iostream>
#include <string>

namespace roscar
{
namespace car
{
namespace roscar_common
{

using ReturnType = std::tuple<int, std::string>;

typedef struct _Camera
{
    std::string uri;

    template <typename T>
    _Camera(T _uri)
    {
        uri = _uri;
    }

    const _Camera &operator=(const _Camera &src)
    {
        uri = src.uri;
    }
} Camera_t;

typedef struct _CameraComparator
{
    bool operator()(const Camera_t &a,
                    const Camera_t &b) const
    {
        return a.uri.compare(b.uri);
    }
} CameraComparator_t;

typedef struct _Video
{
    std::string nodeId;
    std::string videoId;

    template <typename T>
    _Video(T _nodeId, T _videoId)
    {
        nodeId = _nodeId;
        videoId = _videoId;
    }

    const _Video &operator=(const _Video &src)
    {
        nodeId = src.nodeId;
        videoId = src.videoId;
    }
} Video_t;

typedef struct _VideoComparator
{
    bool operator()(const Video_t &a,
                    const Video_t &b) const
    {
        return std::make_pair(a.nodeId, a.nodeId) <
               std::make_pair(b.nodeId, b.nodeId);
    }
} VideoComparator_t;

typedef struct _IPv4
{
    union {
        uint32_t address;
        uint8_t addr[4];
        in_addr inAddr;
    };

    const char *str() { return inet_ntoa(inAddr); }
} IPv4_t;

inline std::ostream &operator<<(std::ostream &out, _IPv4 &ip)
{
    out << ip.str();
    return out;
}

typedef struct _RouteEntry
{
    std::string interface;
    IPv4_t destination;
    IPv4_t gateway;
    int flags;
    int refCnt;
    int use;
    int metric;
    IPv4_t mask;
    int mtu;
    int window;
    int irtt;
} RouteEntry_t;

typedef struct _IntfEntry
{
    std::string interface;
    IPv4_t address;
    IPv4_t mask;
    IPv4_t broadcast;
} IntfEntry_t;

} // namespace roscar_common
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_ROSCARCOMMON_DEFAULT_H_