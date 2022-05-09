#include "ipt_ros_streamer.h"

#include <fcntl.h>

using namespace ipt;

IPT_Streamer::IPT_Streamer()
{
	this->nh.reset(new ros::NodeHandle());
	this->pnh.reset(new ros::NodeHandle("~"));
	this->rate.reset(new ros::Rate(120));

	sock = 0;
	frameStamp = 0;
	client = -1;

	this->ReadParameters();
	this->InitSocket();

	compressionArgs.push_back(cv::IMWRITE_PNG_COMPRESSION);
	compressionArgs.push_back(1);

	isBlocking = true;
}

IPT_Streamer::~IPT_Streamer()
{
	// Let unique_ptrs destroy themselves
	if (sock > 0)
		close(sock);
}

ros::NodeHandle* ipt::IPT_Streamer::GetPrivateNH()
{
	return this->pnh.get();
}

void IPT_Streamer::ReadParameters()
{
	int tport;
	pnh->param<int>("TCPLocalPort", tport, 19810);
	tcpLocalPort = tport;
	pnh->param<int>("TCPRemotePort", tport, 19198);
	tcpRemotePort = tport;
	pnh->param<std::string>("TCPLocalIP", tcpLocalIp, "127.0.0.1");
	pnh->param<std::string>("TCPRemoteIP", tcpRemoteIp, "127.0.0.1");
}

void IPT_Streamer::InitSocket()
{
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		ROS_FATAL("Cannot create TCP socket, %s", strerror(errno));
		ros::shutdown();
	}

	sockaddr_in addr;
	memset(&addr, 0x00, sizeof addr);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(this->tcpLocalPort);
	addr.sin_addr.s_addr = inet_addr(this->tcpLocalIp.c_str());

	ROS_INFO_STREAM("Trying to bind to tcp://" << tcpLocalIp << ":" << tcpLocalPort);

	int ret = bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof addr);
	if (ret)
	{
		ROS_FATAL("Cannot bind socket, %s", strerror(errno));
		ros::shutdown();
	}
	ROS_INFO("Socket bound successful");

	ret = listen(sock, 0);
	if (ret)
	{
		ROS_FATAL("Failed to listen on socket, %s", strerror(errno));
		ros::shutdown();
	}
}

void IPT_Streamer::SetBlock()
{
	int flags = fcntl(sock, F_GETFL, 0);
	if (flags == -1)
	{
		ROS_FATAL("Failed to get file control flags, %s", strerror(errno));
		ros::shutdown();
	}
	int ret = fcntl(sock, F_SETFL, flags & (~O_NONBLOCK));
	if (ret == -1)
	{
		ROS_FATAL("Failed to switch to blocking mode, %s", strerror(errno));
		ros::shutdown();
	}
	isBlocking = true;
}

void IPT_Streamer::SetNonblock()
{
	int flags = fcntl(sock, F_GETFL, 0);
	if (flags == -1)
	{
		ROS_FATAL("Failed to get file control flags, %s", strerror(errno));
		ros::shutdown();
	}
	int ret = fcntl(sock, F_SETFL, flags | O_NONBLOCK);
	if (ret == -1)
	{
		ROS_FATAL("Failed to switch to non-blocking mode, %s", strerror(errno));
		ros::shutdown();
	}
	isBlocking = false;
}

void IPT_Streamer::Loop()
{
	uchar buf[512];
	cv::Mat frame[3];
	sockaddr_in clientAddr;
	socklen_t clientAddrLen;
	while (ros::ok())
	{
		if (isBlocking)
			this->SetNonblock();
		clientAddrLen = sizeof clientAddr;
		client = accept(sock, reinterpret_cast<sockaddr *>(&clientAddr), &clientAddrLen);
		if (client < 0)
		{
			if (errno != EAGAIN && errno != EWOULDBLOCK)
				ROS_ERROR("Cannot accept incoming connection, %s", strerror(errno));
			this->WaitAndSpin();
			continue;
		}

		ROS_INFO("Successfully established connection with tcp://%s:%d",
			inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));

		this->SetBlock();
		bool bSocket = true;
		while (bSocket)
		{

			this->videoCapture >> frame[0] >> frame[1] >> frame[2];

			for (int i = 0; i < 3; i++)
			{
				std::vector <uchar> buf;
				cv::imencode(".png", frame[i], buf, compressionArgs);
				// Send a custom EOF marker
				buf.push_back(0x55);
				buf.push_back(0xAA);
				int ret = send(client, buf.data(), buf.size(), 0);

				if (ret < 0)
				{
					ROS_ERROR("Cannot send data, %s", strerror(errno));
					bSocket = false;
					break;
				}

			}

			if (!bSocket)
				continue;

			// Wait for a response to continue
			int ret = recv(client, buf, sizeof buf, 0);
			if (ret < 0)
			{
				ROS_ERROR("Cannot retrieve response : %s", strerror(errno));
				bSocket = false;
			}

			this->WaitAndSpin();
		}
		ROS_WARN("Rolling back to accept new connections");
	}
}

void IPT_Streamer::WaitAndSpin()
{
	this->rate->sleep();
	ros::spinOnce();
}

void ipt::IPT_Streamer::SetVideoCapture(cv::VideoCapture vCapture)
{
	this->videoCapture = vCapture;
}
