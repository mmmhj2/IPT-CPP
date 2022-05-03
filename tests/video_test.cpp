#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

using std::cout;
using std::endl;
constexpr char DEVICENAME[] = "/dev/video0";

using namespace cv;

int main(int argc, char * argv[])
{
	cv::VideoCapture vCapture{ DEVICENAME };

	if (!vCapture.isOpened())
	{
		cout << "Cannot open device \"" << DEVICENAME << "\"" << endl;
		return -1;
	}

	int height, width, fps, channels;
	height = vCapture.get(cv::CAP_PROP_FRAME_HEIGHT);
	width = vCapture.get(cv::CAP_PROP_FRAME_WIDTH);
	fps = vCapture.get(cv::CAP_PROP_FPS);
	channels = vCapture.get(cv::CAP_PROP_CHANNEL);
	
	cout << "Video properties : "
		<< height << "x" << width << " with "
		<< fps << "FPS (" << channels << " channels)" << endl;

	cout << "Setting properites to desired value ..." << endl;

	bool result;
	result = vCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	result &= vCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
	result &= vCapture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	result &= vCapture.set(cv::CAP_PROP_FPS, 120);
	

	if (!result)
		cout << "Failed to set properities" << endl;
	
	cv::Mat image(360, 640, CV_8UC3);
	vCapture >> image;
	cv::imwrite("image.png", image);

	return 0;
}
