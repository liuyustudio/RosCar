#ifndef _ROSCAR_CAR_CLI_CAMERAMGR_H_
#define _ROSCAR_CAR_CLI_CAMERAMGR_H_

#include <vector>
#include "roscar_common/define.h"

namespace roscar
{
namespace car
{
namespace pilot
{

class CamMgr
{
public:
    static const int DEFAULT_STREAM_PORT = 5110;
    static const char * DEFAULT_NODE_ID;
    static const char * DEFAULT_VIDEO_ID;

    CamMgr() = default;
    ~CamMgr() = default;

    bool start();
    void stop();

protected:
    bool isValid(std::string &uri);
    void queryCams(std::vector<std::string> &uriArray);
    roscar_common::ReturnType openCamera(roscar_common::Camera_t &camera);
    void closeCamera(roscar_common::Camera_t &camera);
};

} // namespace pilot
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_CLI_CAMERAMGR_H_
