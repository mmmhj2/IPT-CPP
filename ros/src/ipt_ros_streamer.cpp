#include "ipt_ros_streamer.h"

#include <cinttypes>
#include <fcntl.h>

using namespace ipt;

IPT_Streamer::IPT_Streamer()
{
	/*
	this->nh.reset(new ros::NodeHandle());
	this->pnh.reset(new ros::NodeHandle("~"));
	this->rate.reset(new ros::Rate(120));
	*/

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
/*
ros::NodeHandle* ipt::IPT_Streamer::GetPrivateNH()
{
	return this->pnh.get();
}
*/

void IPT_Streamer::ReadParameters()
{
	int tport;
	pnh->param<int>("TCPLocalPort", tport, 19810);
	tcpLocalPort = tport;
	pnh->param<int>("TCPRemotePort", tport, 19198);
	tcpRemotePort = tport;
	pnh->param<std::string>("TCPLocalIP", tcpLocalIp, "0.0.0.0");
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

	timeval netTimeout;
	netTimeout.tv_sec = 5;
	netTimeout.tv_usec = 0;

	ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<void*>(&netTimeout), sizeof netTimeout);
	if (ret)
	{
		ROS_ERROR("Cannot set receive timeout, the receiving procedure may be blocked");
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
			ros::Time frameTime = ros::Time::now();
			this->videoCapture >> frame[0] >> frame[1] >> frame[2];

			if (frame[0].empty() || frame[1].empty() || frame[2].empty())
			{
				ROS_FATAL("Video stream ended, exiting");
				close(client);
				ros::shutdown();
			}

			// Send the timestamp
			int32_t timeCounter = frameTime.toSec();
			int ret = send(client, &timeCounter, sizeof timeCounter, 0);
			if (ret <= 0)
			{
				if (ret == 0)
					ROS_WARN("Socket peer shutdown during transmission");
				else
					ROS_ERROR("Cannot send data, %s", strerror(errno));
				bSocket = false;
			}

			if (!bSocket)
				continue;

			// Send serialized image data, three frames
			for (int i = 0; i < 3; i++)
			{
				std::vector <uchar> buf;
				//cv::imencode(".png", frame[i], buf, compressionArgs);
				cv::imencode(".bmp", frame[i], buf);

				auto size = buf.size();
				ROS_DEBUG_STREAM("Image " << i << " size " << size);
				int ret = send(client, &size, sizeof size, 0);
				if (ret > 0)
				{
					// End of stream marker
					if(i == 2)
					{
						buf.push_back('E');
						buf.push_back('o');
						buf.push_back('S');
					}
					
					ret = send(client, buf.data(), buf.size(), 0);
				}
					

				if (ret <= 0)
				{
					if (ret == 0)
						ROS_WARN("Socket peer shutdown during transmission");
					else
						ROS_ERROR("Cannot send data, %s", strerror(errno));
					bSocket = false;
					break;
				}

			}

			if (!bSocket)
				continue;

			// Wait for a response to continue
			ret = recv(client, buf, sizeof buf, 0);
			if (ret <= 0)
			{
				if (ret == 0)
					ROS_INFO("Stream socket peer has performed an orderly shutdown");
				else
					ROS_ERROR("Cannot retrieve response : %s", strerror(errno));
				bSocket = false;
			}

			this->WaitAndSpin();
		}
		ROS_INFO("Rolling back to accept new connections");
		close(client);
	}
}
/*
void IPT_Streamer::WaitAndSpin()
{
	this->rate->sleep();
	ros::spinOnce();
}
*/

void ipt::IPT_Streamer::SetVideoCapture(cv::VideoCapture vCapture)
{
	this->videoCapture = vCapture;
}
