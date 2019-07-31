namespace roscar
{
namespace car
{
namespace interface
{

static const int RCMP_FRAMEHEADSIZE = 4;
static const int RCMP_STARTFLAG = 0x5A;
static const int RCMP_VERSION = 1;
static const int RCMP_MIN_SIGNALING_LENGTH = RCMP_FRAMEHEADSIZE;
static const int RCMP_MAX_SIGNALING_LENGTH = 0x8000;
static const int RCMP_MAXPAYLOAD = RCMP_MAX_SIGNALING_LENGTH - RCMP_FRAMEHEADSIZE;

} // namespace interface
} // namespace car
} // namespace roscar
