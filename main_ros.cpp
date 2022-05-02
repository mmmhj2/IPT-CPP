#include <ros/ros.h>
#include <opencv4/opencv2/opencv.hpp>

#include "ipt_receiver_new.h"
#include "ipt_ros_interface.h"

int main(int argc, char * argv[])
{

	ipt::IPT_ROSInterface* pInterface = ipt::IPT_ROSInterface::GetInstance(argc, argv);

	char * dirPtr = get_current_dir_name();
	ROS_INFO_STREAM("Current working directory : " << dirPtr);
	free(dirPtr);

	cv::Mat R_b_c{ cv::Mat::zeros(3, 3, CV_64F) };
	R_b_c.at<double>(0, 1) = -1;
	R_b_c.at<double>(1, 0) = -1;
	R_b_c.at<double>(0, 1) = -1;

	bool bUseMavrosPose;
	int morphOpenSz, morphCloseSz;
	int height, width;
	double scale_f;
	std::string deviceFile, mapFile, camParamFile;
	pInterface->GetPrivateNH()->param<bool>("UseMavrosPose", bUseMavrosPose, false);
	pInterface->GetPrivateNH()->param<int>("MorphOpenSz", morphOpenSz, 4);
	pInterface->GetPrivateNH()->param<int>("MorphCloseSz", morphCloseSz, 12);
	pInterface->GetPrivateNH()->param<int>("VideoHeight", height, 720);
	pInterface->GetPrivateNH()->param<int>("VideoWidth", width, 1280);
	pInterface->GetPrivateNH()->param<double>("VideoScaleFactor", scale_f, 0.5);
	pInterface->GetPrivateNH()->param<std::string>("DeviceFile", deviceFile, "/dev/video0");
	pInterface->GetPrivateNH()->param<std::string>("MapFile", mapFile, "params/map_info_9x9.json");
	pInterface->GetPrivateNH()->param<std::string>("CameraParameterFile", camParamFile, "params/cam_new.json");

	ROS_INFO_STREAM("Camera file set to " << camParamFile);
	ROS_INFO_STREAM("Map file set to " << mapFile);

	// Create video capture
	ROS_INFO("Setting up video capture...");
	cv::VideoCapture vCapture{ deviceFile };
	if (!vCapture.isOpened())
	{
		ROS_FATAL_STREAM("Cannot open video device : " << deviceFile);
		return -1;
	}

	// Set properities
	bool isPropSet = true;
	isPropSet &= vCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	isPropSet &= vCapture.set(cv::CAP_PROP_FRAME_WIDTH, width);
	isPropSet &= vCapture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
	isPropSet &= vCapture.set(cv::CAP_PROP_FPS, 120);
	if (!isPropSet)
	{
		ROS_FATAL("Cannot set properity of the stream");
		return -1;
	}

	// Test existence of parameter files
	ROS_INFO("Testing parameter files...");
	do{
		std::ifstream finTest;
		finTest.open(camParamFile);
		if (!finTest)
		{
			ROS_FATAL_STREAM("Cannot open camera configuration file " << camParamFile);
			return -1;
		}
		finTest.close();
		finTest.open(mapFile);
		if (!finTest)
		{
			ROS_FATAL_STREAM("Cannot open map configuration file " << mapFile);
			return -1;
		}
		finTest.close();
	} while (0);	// Guard with while(0) to eliminate unnecessary declarations


	// Ready to receive and demodulate
	ipt::IPT_Receiver receiver{camParamFile, mapFile, width, height, scale_f};
	receiver.SetMorphKernelSize(morphOpenSz, morphCloseSz);

	Mat f_pre(height, width, CV_8UC3);
	Mat f_now(height, width, CV_8UC3);
	Mat f_nxt(height, width, CV_8UC3);

	zarray_t* detections;
	Vec3d position;
	Vec3d angle;

	ROS_INFO("Ready to demodulate");

	while (ros::ok())
	{
		ros::Time frameTime = ros::Time::now();
		vCapture >> f_pre >> f_now >> f_nxt;
		if (f_pre.empty() || f_now.empty() || f_nxt.empty())
		{
			ROS_WARN("End of the video stream reached, shutting down...");
			ros::shutdown();
		}

		receiver.Demodulate(f_pre, f_now, f_nxt, detections);
		if (!receiver.tag_exist_flag)
		{
			pInterface->WaitAndSpin();
			continue;
		}
		
		ROS_INFO_THROTTLE(1, "Demodulation successful");
		if (!bUseMavrosPose)
		{
			receiver.EstimatePose(detections, position, angle);
		}
		else
		{
			// TODO : Use mavros pose
			geometry_msgs::Quaternion quat;
			pInterface->GetEstimatedPose(quat);
			cv::Mat rotationMat = ipt::quaternion_2_rotation(quat.w, quat.x, quat.y, quat.z);
			rotationMat = rotationMat * R_b_c;
			receiver.EstimatePoseWithOrientation(detections, position, angle, rotationMat);
		}

		geometry_msgs::PoseStamped posePub;
		posePub.header.stamp = frameTime;
		posePub.pose.position.x = position[0];
		posePub.pose.position.y = position[1];
		posePub.pose.position.z = position[2];
		auto quat = ipt::euler_2_quaternion(angle);
		posePub.pose.orientation.w = quat[0];
		posePub.pose.orientation.x = quat[1];
		posePub.pose.orientation.y = quat[2];
		posePub.pose.orientation.z = quat[3];
		pInterface->PublishPose(posePub);

		pInterface->WaitAndSpin();
	}

	return 0;
}
