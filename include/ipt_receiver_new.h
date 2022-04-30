/*
* Author: Li
* File: ipt_receiver_new.h
* Description: Refractored version of ipt::IPTReceiver
*/

#ifndef IPT_RECEIVER_NEW_INCLUDED
#define IPT_RECEIVER_NEW_INCLUDED

#include "tools.h"
#include "opencv4/opencv2/opencv.hpp"
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"

namespace ipt
{
	class IPT_Receiver
	{
	private:
		cv::Mat cam_mtx, cam_dist;
		MapInfo map_info;

		apriltag_detector_t* td;
		apriltag_family_t* tf;

		cv::Mat img_org[3];
		cv::Mat img_lightness[3];

		int height, width;
		double scale_f_params, scale_factor;
		
	public:
		bool tag_exist_flag;

	private:
		static void
			GetLightnessCh(const cv::Mat& img_bgr, cv::Mat& img_lightness);
		// Align two img
		std::pair<int, int> AlignImg(const cv::Mat& img_lig_pre,
			const cv::Mat& img_lig_new, int nSample = 3);
		// Process the img_sub before detection
		// Including normalizing, blurring, thresholding and morph operations
		void Preprocess(cv::Mat& img_sub);

		// Align two matrixs and find their substraction
		cv::Mat Substraction(cv::Mat& img_lightness_1, cv::Mat& img_lightness_2);
		// Search for Apriltags in img
		int Detection(const cv::Mat & img, zarray_t*& detections);

	public:
		IPT_Receiver(const std::string& cam_path,
			const std::string& map_path,
			int w, int h,
			double scale_f = 1);
		~IPT_Receiver();

		void Demodulate(const cv::Mat& img_pre, 
			const cv::Mat& img_now, 
			const cv::Mat& img_nxt, 
			zarray_t*& detections);

	};
}

#endif