#ifndef IPT_ROS_POSE_RECEIVER_H
#define IPT_ROS_POSE_RECEIVER_H

#include "ipt_ros_base.h"

#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>

namespace ipt
{
	class IPT_PoseReceiver : public IPT_ROSBase
	{
	protected:

		std::string poseNodeName;
		std::string udpLocalIp;
		int udpLocalPort;

		int sock;

		void InitSocket();

	public:

		IPT_PoseReceiver();
		virtual ~IPT_PoseReceiver();

		void Loop();
	};
}

#endif
