#include "ipt_ros_pose_receiver.h"

#include "socket_data.h"
#include <fcntl.h>

using namespace ipt;

IPT_PoseReceiver::IPT_PoseReceiver()
{
	pnh->param<std::string>("PoseTopicName", this->poseNodeName, "ipt/pose");
	pnh->param<std::string>("UdpLocalIp", this->udpLocalIp, "0.0.0.0");
	pnh->param<int>("UdpLocalPort", this->udpLocalPort, 19198);

	this->sock = -1;

	this->InitSocket();
}

IPT_PoseReceiver::~IPT_PoseReceiver()
{
	if (sock >= 0)
		close(sock);
}

void IPT_PoseReceiver::InitSocket()
{
	// TODO : Refactor to reuse pose_receiver
	this->sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
	{
		ROS_FATAL("Cannot create UDP socket, %s", strerror(errno));
		ros::shutdown();
	}

	sockaddr_in addr_in;
	memset(&addr_in, 0x00, sizeof addr_in);
	addr_in.sin_family = AF_INET;
	addr_in.sin_port = htons(udpLocalPort);
	addr_in.sin_addr.s_addr = inet_addr(udpLocalIp.c_str());

	ROS_INFO_STREAM("Trying to bind to udp://" << udpLocalIp << ":" << udpLocalPort);
	int ret = bind(sock, reinterpret_cast<sockaddr*>(&addr_in), sizeof addr_in);
	if (ret)
	{
		ROS_FATAL("Cannot bind socket, %s", strerror(errno));
		ros::shutdown();
	}

	// Set to nonblock mode
	int flags = fcntl(sock, F_GETFL, 0);
	if (flags == -1)
	{
		ROS_FATAL("Failed to get file control flags, %s", strerror(errno));
		ros::shutdown();
	}
	ret = fcntl(sock, F_SETFL, flags | O_NONBLOCK);
	if (ret == -1)
	{
		ROS_FATAL("Failed to switch to non-blocking mode, %s", strerror(errno));
		ros::shutdown();
	}
}

void IPT_PoseReceiver::Loop()
{
	char buffer[512];
	socklen_t addr_len;
	sockaddr_in addr_in;

	while (ros::ok())
	{
		int ret = recvfrom(sock, buffer, sizeof buffer, 0, reinterpret_cast<sockaddr *>(&addr_in), &addr_len);
		if (ret <= 0)
		{
			if (errno == EAGAIN || errno == EWOULDBLOCK)
			{
				this->WaitAndSpin();
				continue;
			}
			ROS_FATAL("Cannot receive from UDP socket, %s", strerror(errno));
			ros::shutdown();
		}

		ROS_INFO_THROTTLE(1, "Received %d bytes from udp://%s:%d", ret, inet_ntoa(addr_in.sin_addr), ntohs(addr_in.sin_port));
		
		this->WaitAndSpin();
	}
}
