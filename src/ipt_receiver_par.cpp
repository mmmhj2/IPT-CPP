/*
* Author: Li
* File: ipt_receiver_par.cpp
* Description: Refractored version of ipt::IPTReceiver, using C++11 concurrency support
*/
#include "ipt_receiver_par.h"
#include <future>

void ipt::IPT_Receiver_Parallel::CopyAndConvert(int index, const cv::Mat& from)
{
	this->img_org[index] = from;
	if (scale_factor != 1.0)
		cv::resize(img_org[index], img_org[index], cv::Size(),
			scale_factor, scale_factor, INTER_LINEAR);
	GetLightnessCh(img_org[index], img_lightness[index]);
}

int ipt::IPT_Receiver_Parallel::Detection(const cv::Mat& img, zarray_t*& detections)
{
	apriltag_detector_t * pDetector = static_cast<apriltag_detector_t * >(malloc(sizeof(apriltag_detector_t)));
	memcpy(pDetector, this->td, sizeof(apriltag_detector_t));
	image_u8_t img_u8 = {
		.width = img.cols,
		.height = img.rows,
		.stride = img.cols,
		.buf = img.data };
	detections = apriltag_detector_detect(pDetector, &img_u8);
	free(pDetector);

	return zarray_size(detections);
}

ipt::IPT_Receiver_Parallel::IPT_Receiver_Parallel(const std::string& cam_path, const std::string& map_path, int w, int h, double scale_f)
	: IPT_Receiver(cam_path, map_path, w, h, scale_f)
{
	if (std::thread::hardware_concurrency() <= 1)
		throw std::runtime_error("No hardware concurrency on this computer");
}

ipt::IPT_Receiver_Parallel::~IPT_Receiver_Parallel()
{
}

void ipt::IPT_Receiver_Parallel::Demodulate(const cv::Mat& img_pre, const cv::Mat& img_now, const cv::Mat& img_nxt, zarray_t*& detections)
{
	tag_exist_flag = false;
	auto copy_and_convert_1 = std::async(
		std::mem_fn(&IPT_Receiver_Parallel::CopyAndConvert), this, 0, img_pre);
	auto copy_and_convert_2 = std::async(
		std::mem_fn(&IPT_Receiver_Parallel::CopyAndConvert), this, 1, img_now);
	auto copy_and_convert_3 = std::async(
		std::mem_fn(&IPT_Receiver_Parallel::CopyAndConvert), this, 2, img_nxt);
	copy_and_convert_1.wait();
	copy_and_convert_2.wait();
	copy_and_convert_3.wait();


	cv::Mat sub1, sub2;
	sub1 = this->Substraction(img_lightness[0], img_lightness[1]);
	sub2 = this->Substraction(img_lightness[1], img_lightness[2]);
	// Use std::ref to pass a reference
	auto preprocess_1 = std::async(
		std::mem_fn(&IPT_Receiver_Parallel::Preprocess), this, std::ref(sub1));
	auto preprocess_2 = std::async(
		std::mem_fn(&IPT_Receiver_Parallel::Preprocess), this, std::ref(sub2));
	preprocess_1.wait();
	preprocess_2.wait();

	cv::Mat sub1_inverse, sub2_inverse;
	sub1_inverse = 255 - sub1;
	sub2_inverse = 255 - sub2;

	cv::Mat* mRefs[4] = { &sub1, &sub1_inverse, &sub2, &sub2_inverse };
	zarray_t* dets[4] = { nullptr, nullptr, nullptr, nullptr };
	std::future<int> detectionFutures[4];

	for (int i = 0; i < 4; i++)
		detectionFutures[i] = std::async(
			std::mem_fn(&IPT_Receiver_Parallel::Detection),
			this,
			std::cref(*mRefs[i]),
			std::ref(dets[i])
		);

	for (int i = 0; i < 4; i++)
	{
		int ret = detectionFutures[i].get();
		if (ret > 0)
		{
			tag_exist_flag = true;
			detections = zarray_copy(dets[i]);
			break;
		}
	}

	for (int i = 0; i < 4; i++)
		zarray_destroy(dets[i]);
}
