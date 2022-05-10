#include <opencv4/opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <poll.h>

constexpr char HOST[] = "127.0.0.1";
constexpr short PORT = 19810;

using std::cout;
using std::endl;

// A huge 16MiB buffer enough for three images
uchar buffer[16 * 1024 * 1024];

int main(int argc, char** argv)
{
	char ACK[] = "ACK";
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
		for (ptr = buffer; ptr - buffer < 4; ++ptr)
			data[0].push_back(*ptr);

		// Split the images
		for (int counter = 1; counter <= 3; counter++)
		{
			std::vector <uchar> sizeBuf;
			//sizeBuf.insert(sizeBuf.end(), ptr, ptr + sizeof sizeBuf.size());
			//ptr += sizeof sizeBuf.size();
			uchar* nptr;
			for (nptr = ptr; nptr - ptr < sizeof(size_t); ++nptr)
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

			cv::imshow("Image", frame[i-1]);
			cv::waitKey(1);
		}
			
		

	}
	close(sock);
	return 0;
}
