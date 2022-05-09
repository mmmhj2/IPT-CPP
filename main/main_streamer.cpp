#include "ipt_ros_streamer.h"

int main(int argc, char* argv[])
{
	int height, width;
	std::string deviceFile;

	ros::init(argc, argv, "ipt_streamer_node");
	ipt::IPT_Streamer Streamer;

	auto pnh = Streamer.GetPrivateNH();
	pnh->param<int>("VideoHeight", height, 720);
	pnh->param<int>("VideoWidth", width, 1280);
	pnh->param<std::string>("DeviceFile", deviceFile, "/dev/video0");

	cv::VideoCapture videoCapture{ deviceFile };
	if (!videoCapture.isOpened())
	{
		ROS_FATAL_STREAM("Cannot open video device : " << deviceFile);
		return -1;
	}

	bool isPropSet = true;
	isPropSet &= videoCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	isPropSet &= videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, width);
	isPropSet &= videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
	isPropSet &= videoCapture.set(cv::CAP_PROP_FPS, 120);
	if (!isPropSet)
		ROS_ERROR("Cannot set properity of the stream, results may be undesirable");

	Streamer.SetVideoCapture(videoCapture);

	Streamer.Loop();
}
