#ifndef _ROSCAR_CAR_ROSCARCOMMON_ERROR_H_
#define _ROSCAR_CAR_ROSCARCOMMON_ERROR_H_

namespace roscar
{
namespace car
{
namespace roscar_common
{

static const int SUCCESS = 0;
static const int ERROR_SUCCESS = SUCCESS;
static const int FAIL = -1;
static const int NEED_MORE_DATA = -2;
static const int ERROR = FAIL;
static const int ERROR_INVALID = -10;
static const int ERROR_UNSUPPORT = -11;

} // namespace roscar_common
} // namespace car
} // namespace roscar

#endif // _ROSCAR_CAR_ROSCARCOMMON_ERROR_H_