#ifndef _ROSCAR_CAR_INTERFACE_INTERFACE_H_
#define _ROSCAR_CAR_INTERFACE_INTERFACE_H_

#include <map>
#include "rapidjson/document.h"
#include "uds.h"

namespace roscar
{
namespace car
{
namespace interface
{

class Interface
{
protected:
    typedef bool (*FUNC_ONSIG)(UDS::SESSION_t &sess, rapidjson::Document &sig);

    Interface(){};

public:
    // overide
    static bool onSignaling(UDS::SESSION_t &sess, rapidjson::Document &sig);

    // init Interface runtime environment
    static void init();

protected:
    static bool onSigPing(UDS::SESSION_t &sess, rapidjson::Document &sig);
    static bool onSigPong(UDS::SESSION_t &sess, rapidjson::Document &sig);
    static bool onSigInfo(UDS::SESSION_t &sess, rapidjson::Document &sig);

    static bool sendSignaling(UDS::SESSION_t &sess, rapidjson::Document &sig);

    static std::map<const char *, FUNC_ONSIG> mSigFuncMap;
};

} // namespace interface
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_INTERFACE_INTERFACE_H_
