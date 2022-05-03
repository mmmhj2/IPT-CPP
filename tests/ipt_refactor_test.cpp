/*
* Author: Li
* File: ipt_refactor_test.cpp
* Description: Unit test for original and refactored IPT
*/

#include "ipt_receiver_new.h"
#include "ipt_receiver_par.h"
#include "ipt_receiver.h"

#include <string>
#include <chrono>
#include <opencv4/opencv2/opencv.hpp>

using namespace cv;

constexpr char CAM_PARA_PATH[] = "../../../params/cam_para_80d_1280x720.json";
constexpr char MAP_PARA_PATH[] = "../../../params/map_info_9x9.json";
constexpr char VIDEONAME[] = "../../../videos/0530_color_120fps_L=4_9x9_noLight_720p.avi";
constexpr double SCALE_F = 0.5;
constexpr int WIDTH = 1280;
constexpr int HEIGHT = 720;

int main(int argc, char* argv[])
{
	std::ofstream detectionLog;
	std::cerr << get_current_dir_name() << std::endl;
	cv::VideoCapture vCap{VIDEONAME};
	
	if (!vCap.isOpened())
	{
		std::cerr << "Cannot open video " << VIDEONAME << std::endl;
		return -1;
	}

	cv::Mat f_pre(HEIGHT, WIDTH, CV_8UC3);
	cv::Mat f_now(HEIGHT, WIDTH, CV_8UC3);
	cv::Mat f_nxt(HEIGHT, WIDTH, CV_8UC3);
	zarray_t* detections;
	Vec3d position, angle;
	int SuccessfulCnt{ 0 }, frameCnt{ 1 };

	ipt::IPTReceiver receiver(
		CAM_PARA_PATH,
		MAP_PARA_PATH,
		WIDTH,
		HEIGHT,
		SCALE_F,
		true);

	detectionLog.open("original.log");


	auto start_timer = std::chrono::steady_clock::now();
	while (SuccessfulCnt < 100)
	{
		frameCnt += 3;
		vCap >> f_pre >> f_now >> f_nxt;
		if (f_nxt.empty() || f_now.empty() || f_pre.empty())
		{
			std::cout << "The end of the video has been reached, but no estimation is succeeded" << std::endl;
			break;
		}

		receiver.demodulate(f_pre, f_now, f_nxt, detections);
		receiver.estimate_pose(detections, position, angle);

		if (receiver.tag_exist_flag)
		{
			//printf("Successful at frame %d, (%f, %f, %f), (%f, %f, %f)\n",
			//	frameCnt, position[0], position[1], position[2], angle[0], angle[1], angle[2]);
			detectionLog << "At frame " << frameCnt << " "
				<< "(" << position[0] << "," << position[1] << "," << position[2] << ") "
				<< "(" << angle[0] << "," << angle[1] << "," << angle[2] << ")" << std::endl;
			SuccessfulCnt++;
		}
	}
	auto end_timer = std::chrono::steady_clock::now();
	detectionLog << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() << " ms" << std::endl;
	detectionLog.close();

	// Refactored
	detectionLog.open("refactored.log");

	vCap.open(VIDEONAME);
	SuccessfulCnt = 0, frameCnt = 1;
	ipt::IPT_Receiver refactored(CAM_PARA_PATH, MAP_PARA_PATH, WIDTH, HEIGHT, SCALE_F);

	start_timer = std::chrono::steady_clock::now();
	while (SuccessfulCnt < 100)
	{
		frameCnt += 3;
		vCap >> f_pre >> f_now >> f_nxt;
		if (f_nxt.empty() || f_now.empty() || f_pre.empty())
		{
			std::cout << "The end of the video has been reached, but no estimation is succeeded" << std::endl;
			break;
		}

		refactored.Demodulate(f_pre, f_now, f_nxt, detections);
		refactored.EstimatePose(detections, position, angle);

		if (refactored.tag_exist_flag)
		{
			//printf("Successful at frame %d, (%f, %f, %f), (%f, %f, %f)\n",
			//	frameCnt, position[0], position[1], position[2], angle[0], angle[1], angle[2]);
			detectionLog << "At frame " << frameCnt << " "
				<< "(" << position[0] << "," << position[1] << "," << position[2] << ") "
				<< "(" << angle[0] << "," << angle[1] << "," << angle[2] << ")" << std::endl;
			SuccessfulCnt++;
		}
	}
	end_timer = std::chrono::steady_clock::now();
	detectionLog << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() << " ms" << std::endl;
	detectionLog.close();
#ifdef TEST_PARALLEL
	// Refactored parallel
	detectionLog.open("refactored_par.log");

	vCap.open(VIDEONAME);
	SuccessfulCnt = 0, frameCnt = 1;
	ipt::IPT_Receiver_Parallel refactored_par(CAM_PARA_PATH, MAP_PARA_PATH, WIDTH, HEIGHT, SCALE_F);

	start_timer = std::chrono::steady_clock::now();
	while (SuccessfulCnt < 100)
	{
		frameCnt += 3;
		vCap >> f_pre >> f_now >> f_nxt;
		if (f_nxt.empty() || f_now.empty() || f_pre.empty())
		{
			std::cout << "The end of the video has been reached, but no estimation is succeeded" << std::endl;
			break;
		}

		refactored_par.Demodulate(f_pre, f_now, f_nxt, detections);
		refactored_par.EstimatePose(detections, position, angle);

		if (refactored_par.tag_exist_flag)
		{
			//printf("Successful at frame %d, (%f, %f, %f), (%f, %f, %f)\n",
			//	frameCnt, position[0], position[1], position[2], angle[0], angle[1], angle[2]);
			detectionLog << "At frame " << frameCnt << " "
				<< "(" << position[0] << "," << position[1] << "," << position[2] << ") "
				<< "(" << angle[0] << "," << angle[1] << "," << angle[2] << ")" << std::endl;
			SuccessfulCnt++;
		}
	}
	end_timer = std::chrono::steady_clock::now();
	detectionLog << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() << " ms" << std::endl;
	detectionLog.close();
#endif
	return 0;
}
