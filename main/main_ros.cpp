#include <ros/ros.h>
#include <opencv4/opencv2/opencv.hpp>

#include "ipt_receiver_new.h"
#include "ipt_ros_interface.h"

int main(int argc, char * argv[])
{

	ros::init(argc, argv, "ipt_node");

	ipt::IPT_ROSInterface* pInterface = ipt::IPT_ROSInterface::GetInstance();

	char * dirPtr = get_current_dir_name();
	ROS_INFO_STREAM("Current working directory : " << dirPtr);
	free(dirPtr);

	cv::Mat R_b_c{ cv::Mat::zeros(3, 3, CV_64F) };
	R_b_c.at<double>(0, 1) = -1;
	R_b_c.at<double>(1, 0) = -1;
	R_b_c.at<double>(2, 2) = -1;

	bool bUseMavrosPose, bLogResult;
	int morphOpenSz, morphCloseSz;
	int height, width;
	double scale_f;
	std::string deviceFile, mapFile, camParamFile, loggingFile;
	pInterface->GetPrivateNH()->param<bool>("LoggingResults", bLogResult, true);
	pInterface->GetPrivateNH()->param<bool>("UseMavrosPose", bUseMavrosPose, false);
	pInterface->GetPrivateNH()->param<int>("MorphOpenSz", morphOpenSz, 4);
	pInterface->GetPrivateNH()->param<int>("MorphCloseSz", morphCloseSz, 12);
	pInterface->GetPrivateNH()->param<int>("VideoHeight", height, 720);
	pInterface->GetPrivateNH()->param<int>("VideoWidth", width, 1280);
	pInterface->GetPrivateNH()->param<double>("VideoScaleFactor", scale_f, 0.5);
	pInterface->GetPrivateNH()->param<std::string>("DeviceFile", deviceFile, "/dev/video0");
	pInterface->GetPrivateNH()->param<std::string>("MapFile", mapFile, "params/map_info_9x9.json");
	pInterface->GetPrivateNH()->param<std::string>("CameraParameterFile", camParamFile, "params/cam_new.json");
	pInterface->GetPrivateNH()->param<std::string>("LoggingFile", loggingFile, "results.log");

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

	cv::Mat f_pre(height, width, CV_8UC3);
	cv::Mat f_now(height, width, CV_8UC3);
	cv::Mat f_nxt(height, width, CV_8UC3);

	zarray_t* detections;
	cv::Vec3d position;
	cv::Vec3d angle;
	cv::Mat rotationMat;

	std::ofstream flog;
	if (bLogResult)
	{
		flog.open(loggingFile);
		if (!flog.good())
		{
			ROS_ERROR_STREAM("Cannot open logging file \"" << loggingFile << "\", no logging is provided.");
			bLogResult = false;
		}
		else
		{
			if (!bUseMavrosPose)
				flog << "X_RAW, Y_RAW, Z_RAW" << std::endl;
			else
				flog << "X, Y, Z, X_RAW, Y_RAW, Z_RAW, X_FILTERED, Y_FILTERED, Z_FILTERED" << std::endl;
		}
	}
		

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

		geometry_msgs::PoseStamped posePub;
		cv::Vec4d quat;

		if (!bUseMavrosPose)
		{
			cv::Mat rotMat;
			receiver.EstimatePose(detections, position, rotMat);

			posePub.header.stamp = frameTime;
			posePub.pose.position.x = position[0];
			posePub.pose.position.y = position[1];
			posePub.pose.position.z = position[2];
			quat = ipt::rotation_2_quaternion(rotMat);
			posePub.pose.orientation.w = quat[0];
			posePub.pose.orientation.x = quat[1];
			posePub.pose.orientation.y = quat[2];
			posePub.pose.orientation.z = quat[3];
			pInterface->PublishPose(posePub);

			if (bLogResult)
				flog << position[0] << "," << position[1] << "," << position[2] << std::endl;
		}
		else
		{
			static bool isFirstDemod = true;
			static cv::Mat alignMat;

			std_msgs::Header ros_header;
			geometry_msgs::Quaternion ros_quat;
			geometry_msgs::PoseStamped pose_raw, pose_uncali;
			cv::Vec3d posr, angler, posUncal;

			pInterface->GetEstimatedPose(ros_quat, ros_header);

			if (std::abs((ros_header.stamp - frameTime).toSec()) > 1)
			{
				ROS_WARN("Discarding demodulation due to high delay.");
				pInterface->WaitAndSpin();
				continue;
			}

			// Align the NEU frame with local frame
			if (isFirstDemod)
			{
				isFirstDemod = false;
				cv::Vec4d cv_quat{ ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z };
				alignMat = ipt::AlignRotationMatrix(cv_quat);
			}

			rotationMat = ipt::quaternion_2_rotation(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);
			rotationMat = alignMat * rotationMat;
			//cv::transpose(R_b_c * rotationMat, rotationMat);
			rotationMat = rotationMat * R_b_c;
			receiver.EstimatePoseWithOrientation(detections, position, rotationMat, posr, angler, posUncal);

			auto newQuat = ipt::rotation_2_quaternion(rotationMat);
			ros_quat.w = newQuat[0];
			ros_quat.x = newQuat[1];
			ros_quat.y = newQuat[2];
			ros_quat.z = newQuat[3];

			// Set the pose, calibrated by the rotation filtered by flight control system
			posePub.header.stamp = frameTime;
			pose_raw.header.stamp = frameTime;
			posePub.pose.position.x = position[0];
			posePub.pose.position.y = position[1];
			posePub.pose.position.z = position[2];
			posePub.pose.orientation = ros_quat;

			// Set the raw pose, calibrated by the rotation calculated by estimated rotation matrix
			quat = ipt::euler_2_quaternion(angler);
			pose_raw.pose.position.x = posr[0];
			pose_raw.pose.position.y = posr[1];
			pose_raw.pose.position.z = posr[2];
			pose_raw.pose.orientation.w = quat[0];
			pose_raw.pose.orientation.x = quat[1];
			pose_raw.pose.orientation.y = quat[2];
			pose_raw.pose.orientation.z = quat[3];

			// Set the raw pose, uncalibrated by the rotation matrix
			pose_uncali.pose.position.x = posUncal[0];
			pose_uncali.pose.position.y = posUncal[1];
			pose_uncali.pose.position.z = posUncal[2];
			pose_uncali.pose.orientation.w = 1;
			pose_uncali.pose.orientation.x = 0;
			pose_uncali.pose.orientation.y = 0;
			pose_uncali.pose.orientation.z = 0;

			pInterface->PublishPose(posePub, pose_raw, pose_uncali);
			pInterface->GetFilteredPose(pose_raw);

			if (bLogResult)
				flog << position[0] << "," << position[1] << "," << position[2] << ","
				<< posr[0] << "," << posr[1] << "," << posr[2] << ","
				<< pose_raw.pose.position.x << "," << pose_raw.pose.position.y << "," << pose_raw.pose.position.z
				<< std::endl ;
		}

		pInterface->WaitAndSpin();
	}

	if (flog.is_open())
		flog.close();

	return 0;
}
