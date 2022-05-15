#include "ipt_receiver_new.h"
#include "socket_data.h"

#include <opencv4/opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <poll.h>

constexpr char HOST[] = "127.0.0.1";
constexpr short TCPPORT = 19810;
constexpr short UDPPORT = 19198;

constexpr char CAM_PARA_PATH[] = "../../../params/cam_para_80d_1280x720.json";
constexpr char MAP_PARA_PATH[] = "../../../params/map_info_9x9.json";
constexpr double SCALE_F = 0.5;
constexpr int WIDTH = 1280;
constexpr int HEIGHT = 720;

using std::cout;
using std::endl;

// A huge 16MiB buffer enough for three images
uchar buffer[16 * 1024 * 1024];

int main(int argc, char** argv)
{
	ipt::IPT_Receiver refactored(CAM_PARA_PATH, MAP_PARA_PATH, WIDTH, HEIGHT, SCALE_F);

	char ACK[] = "ACK";
	int sock = socket(AF_INET, SOCK_STREAM, 0);
	int sockUDP = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in udpServerAddr;
	memset(&udpServerAddr, 0x00, sizeof udpServerAddr);
	udpServerAddr.sin_family = AF_INET;
	udpServerAddr.sin_addr.s_addr = inet_addr(HOST);
	udpServerAddr.sin_port = htons(UDPPORT);

	sockaddr_in serverAddr;
	memset(&serverAddr, 0x00, sizeof serverAddr);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = inet_addr(HOST);
	serverAddr.sin_port = htons(TCPPORT);

	int ret = connect(sock, reinterpret_cast<sockaddr*>(&serverAddr), sizeof serverAddr);
	if (ret < 0)
	{
		cout << "Cannot connect to server : " << strerror(errno) << endl;
		return -1;
	}

	cout << "Connection successfully established" << endl;

	bool bQuit = false;
	cv::Mat frame[3];

	while (!bQuit)
	{
		//memset(buffer, 0x00, sizeof buffer);
		
		std::vector <uchar> data[4];
		
		// Fill up the buffer
		uchar* bufPtr = buffer;
		bool isFinished = false;
		while (!isFinished)
		{
			int bytesRead = recv(sock, bufPtr, buffer + sizeof buffer - bufPtr, 0);
			if (bytesRead <= 0)
			{
				if (bytesRead == 0)
					cout << "Connection shutdown by peer" << endl;
				else
					cout << "Cannot read from TCP connection : " << strerror(errno) << endl;
				return 0;
			}

			bufPtr += bytesRead;
			//cout << "Received " << bytesRead << " bytes" << endl;
			//cout << "Trailing : " << *(bufPtr - 3) << *(bufPtr - 2) << *(bufPtr - 1) << endl;
			if (*(bufPtr - 3) == 'E' && *(bufPtr - 2) == 'o' && *(bufPtr - 1) == 'S')
				isFinished = true;
		}
		int ret = send(sock, ACK, sizeof ACK, 0);

		// Get timestamp
		uchar * ptr = buffer;
		for (ptr = buffer; ptr - buffer < sizeof(TimestampType); ++ptr)
			data[0].push_back(*ptr);

		// Split the images
		for (int counter = 1; counter <= 3; counter++)
		{
			std::vector <uchar> sizeBuf;
			//sizeBuf.insert(sizeBuf.end(), ptr, ptr + sizeof sizeBuf.size());
			//ptr += sizeof sizeBuf.size();
			uchar* nptr;
			for (nptr = ptr; nptr - ptr < sizeof(ImageSzType); ++nptr)
				sizeBuf.push_back(*nptr);
			ptr = nptr;

			auto size = *(reinterpret_cast<decltype(sizeBuf.size())*>(sizeBuf.data()));
			//cout << "Expected size of image " << counter << " " << size << endl;
			data[counter].insert(data[counter].end(), ptr, ptr + size);
			ptr += size;
		}

		//cout << "Timestamp : " << *(reinterpret_cast<int*>(data[0].data())) << endl;
		for (int i = 1; i <= 3; i++)
		{
			//cout << "Image " << i << " : " << data[i].size() << " bytes" << endl;
			frame[i-1] = cv::imdecode(data[i], cv::IMREAD_COLOR);

			//cout << frame[i - 1].cols << "x" << frame[i - 1].rows << endl;
			//cv::imshow("Image", frame[i-1]);
			//cv::waitKey(1);
		}
		
		zarray_t * detections;
		cv::Vec3d position, angle;
		cv::Mat rotMat;
		refactored.Demodulate(frame[0], frame[1], frame[2], detections);
		refactored.EstimatePose(detections, position, rotMat);
		angle = ipt::rotation_2_euler(rotMat);

		if (refactored.tag_exist_flag)
		{
			cout << "Estimation successful" << endl;
			cout << "Position : " << position << endl;
			//int ret = sendto(sockUDP, ACK, sizeof ACK, 0, reinterpret_cast<sockaddr*>(&udpServerAddr), sizeof udpServerAddr);
			SocketPose sp;
			sp.stamp = *reinterpret_cast<TimestampType*>(data[0].data());
			sp.x = position[0];
			sp.y = position[1];
			sp.z = position[2];

			cv::Vec4d quat = ipt::euler_2_quaternion(angle);
			sp.qw = quat[0];
			sp.qx = quat[1];
			sp.qy = quat[2];
			sp.qz = quat[3];
			int ret = sendto(sockUDP, &sp, sizeof sp, 0, reinterpret_cast<sockaddr*>(&udpServerAddr), sizeof udpServerAddr);
		}
			

	}
	close(sock);
	return 0;
}
