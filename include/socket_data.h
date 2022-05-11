#ifndef SOCKET_DATA_H
#define SOCKET_DATA_H

#include <cstring>
#include <cstdint>

typedef double PoseDataType;
typedef double TimestampType;
typedef size_t ImageSzType;

typedef struct tagSocketPose
{
	TimestampType stamp;
	PoseDataType x, y, z;
	PoseDataType qw, qx, qy, qz;
}SocketPose;

typedef union tagSocketPoseConverter
{
	SocketPose data;
	char binary[sizeof data];
}SocketPoseConverter;

SocketPose GetDeserializedPose(const char* buf);

#endif
