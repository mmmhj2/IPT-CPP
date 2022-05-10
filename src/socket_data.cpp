#include "socket_data.h"

SocketPose GetDeserializedPose(const char* buf)
{
	static_assert(sizeof(SocketPose) == sizeof(SocketPoseConverter), "Converter size assertion failed");
	static SocketPoseConverter cvt;
	std::memcpy(&cvt, buf, sizeof cvt);
	return cvt.data;
}
