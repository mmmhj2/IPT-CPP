#include "socket_data.h"

typedef union tagSocketPoseConverter
{
	SocketPose data;
	char binary[sizeof data];
}SocketPoseConverter;

SocketPose GetDeserializedPose(const char* buf)
{
	static_assert(sizeof(SocketPose) == sizeof(SocketPoseConverter));
	static SocketPoseConverter cvt;
	std::memcpy(&cvt, buf, sizeof cvt);
	return cvt.data;
}
