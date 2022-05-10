#ifndef IPT_ROS_STREAMER_H
#define IPT_ROS_STREAMER_H

#include "ipt_ros_base.h"

#include <opencv4/opencv2/opencv.hpp>
#include <string>

#include <sys/socket.h>
#include <arpa/inet.h>

namespace ipt
{
	class IPT_Streamer : public IPT_ROSBase
	{
	protected:

		//std::unique_ptr <ros::NodeHandle> nh, pnh;
		//std::unique_ptr <ros::Rate> rate;

		uchar frameStamp;
		uint32_t tcpLocalPort, tcpRemotePort;
		std::string tcpLocalIp, tcpRemoteIp;

		std::vector <int> compressionArgs;
		
		cv::VideoCapture videoCapture;

		int sock;
		int client;

		void ReadParameters();
		void InitSocket();

		bool isBlocking;
		void SetNonblock();
		void SetBlock();

	public:

		IPT_Streamer();
		virtual ~IPT_Streamer();

		// Remove copy sematics due to std::unique_ptr
		IPT_Streamer(const IPT_Streamer&) = delete;
		IPT_Streamer& operator = (const IPT_Streamer&) = delete;

		//ros::NodeHandle* GetPrivateNH();

		void Loop();
		//void WaitAndSpin();

		void SetVideoCapture(cv::VideoCapture vCapture);
		
	};
}

#endif // IPT_ROS_STREAMER_H
