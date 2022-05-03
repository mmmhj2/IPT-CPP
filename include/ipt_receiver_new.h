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
	protected:

		constexpr static double PRJ_SCALE_F = 0.08 / 0.18;

		cv::Mat cam_mtx, cam_dist;
		MapInfo map_info;

		apriltag_detector_t* td;
		apriltag_family_t* tf;

		cv::Mat img_org[3];
		cv::Mat img_lightness[3];

		uint32_t openSz, closeSz;
		uint32_t height, width;
		double scale_f_params, scale_factor;
		
	public:
		bool tag_exist_flag;

	protected:
		static void
			GetLightnessCh(const cv::Mat& img_bgr, cv::Mat& img_lightness);
		// Align two img
		std::pair<int, int> AlignImg(const cv::Mat& img_lig_pre,
			const cv::Mat& img_lig_new, int nSample = 3);

		std::pair <cv::Mat, cv::Mat> GetRTVector(zarray_t *& detections);
		// Process the img_sub before detection
		// Including normalizing, blurring, thresholding and morph operations
		void Preprocess(cv::Mat& img_sub);

		// Align two matrixs and find their substraction
		cv::Mat Substraction(cv::Mat& img_lightness_1, cv::Mat& img_lightness_2);
		// Search for Apriltags in img
		int Detection(const cv::Mat & img, zarray_t*& detections);
		// Search for the coordinate of the given apriltag
		void LookupMap(int index_num, int idx, cv::Mat& obj_pts);

		

	public:
		IPT_Receiver(const std::string& cam_path,
			const std::string& map_path,
			uint32_t w, uint32_t h,
			double scale_f = 1);
		virtual ~IPT_Receiver();

		void SetMorphKernelSize(uint32_t open, uint32_t close);

		virtual void Demodulate(const cv::Mat& img_pre, 
			const cv::Mat& img_now, 
			const cv::Mat& img_nxt, 
			zarray_t*& detections);

		virtual void EstimatePose(zarray_t*& detections, 
			cv::Vec3d& position, 
			cv::Vec3d& angle);

		// Estimate the pose, but use an orientation supplied externally
		virtual void EstimatePoseWithOrientation(zarray_t*& detections,
			cv::Vec3d& position,
			cv::Vec3d& angle,
			const cv::Mat& rotationMat);

		// Estimate the pose with an orientation supplied externally
		// and also output pose estimated with its own orientation 
		virtual void EstimatePoseWithOrientation(zarray_t*& detections,
			cv::Vec3d& position,
			cv::Vec3d& angle,
			const cv::Mat& rotationMat,
			cv::Vec3d& positionRaw,
			cv::Vec3d& angleRaw);
	};
}

#endif
