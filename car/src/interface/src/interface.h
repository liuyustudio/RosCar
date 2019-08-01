#ifndef _ROSCAR_CAR_INTERFACE_INTERFACE_H_
#define _ROSCAR_CAR_INTERFACE_INTERFACE_H_

#include "rapidjson/document.h"
#include "roscar_common/rcmp.h"
#include "uds.h"

namespace roscar
{
namespace car
{
namespace interface
{

class Interface : public UDS::SigCallback
{
public:
    // overide
    bool onSignaling(UDS::SESSION_t &sess, rapidjson::Document &sig);

protected:
    bool onSigPing(UDS::SESSION_t &sess, rapidjson::Document &sig);
    bool onSigPong(UDS::SESSION_t &sess, rapidjson::Document &sig);

    bool sendSignaling(UDS::SESSION_t &sess, rapidjson::StringBuffer &buffer);

    roscar_common::RCMP mRcmp;
};

} // namespace interface
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_INTERFACE_H_
