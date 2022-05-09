#include <opencv4/opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <iostream>

constexpr char HOST[] = "127.0.0.1";
constexpr short PORT = 19810;

using std::cout;
using std::endl;

int main(int argc, char** argv)
{
	uchar buffer[2048];
	int sock = socket(AF_INET, SOCK_STREAM, 0);

	sockaddr_in serverAddr;
	memset(&serverAddr, 0x00, sizeof serverAddr);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = inet_addr(HOST);
	serverAddr.sin_port = htons(PORT);

	int ret = connect(sock, reinterpret_cast<sockaddr*>(&serverAddr), sizeof serverAddr);
	if (ret < 0)
	{
		cout << "Cannot connect to server : " << strerror(errno) << endl;
		return -1;
	}

	cout << "Connection successfully established" << endl;

	bool bQuit = false;
	while (!bQuit)
	{
		ret = recv(sock, buffer, sizeof buffer, 0);
		cout << "Received " << ret << " bytes of data" << endl;
	}
	return 0;
}
