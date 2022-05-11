#ifndef IPT_ROS_POSE_RECEIVER_H
#define IPT_ROS_POSE_RECEIVER_H

#include "ipt_ros_base.h"
#include "socket_data.h"

#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <geometry_msgs/PoseStamped.h>

namespace ipt
{
	class IPT_PoseReceiver : public IPT_ROSBase
	{
	protected:

		std::string poseNodeName;
		std::string udpLocalIp;
		std::string frameName;
		
		uint32_t seq;
		int udpLocalPort;

		int sock;

		ros::Publisher posePub;

		void InitSocket();
		geometry_msgs::PoseStamped MakePose(const SocketPose& pose);

	public:

		IPT_PoseReceiver();
		virtual ~IPT_PoseReceiver();

		void Loop();
	};
}

#endif
