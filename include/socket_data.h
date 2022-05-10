#ifndef SOCKET_DATA_H
#define SOCKET_DATA_H

#include <cstring>
#include <cstdint>

typedef struct tagSocketPose
{
	int32_t stamp;
	double x, y, z;
	double qw, qx, qy, qz;
}SocketPose;

SocketPose GetDeserializedPose(const char* buf);

#endif
