/*
* Author: Li
* File: ipt_receiver_new.h
* Description: Refractored version of ipt::IPTReceiver
*/

#include "ipt_receiver_new.h"

void 
ipt::IPT_Receiver::GetLightnessCh(
	const cv::Mat& img_bgr,
	cv::Mat& img_lightness)
{
	cv::Mat tMat;
	std::vector<cv::Mat> channels(3);

	cv::cvtColor(img_bgr, tMat, COLOR_BGR2Lab);
	cv::split(tMat, channels);
	channels[0].convertTo(img_lightness, CV_16SC1);
}

std::pair<int, int> 
ipt::IPT_Receiver::AlignImg(
	const cv::Mat& img_lig_pre,
	const cv::Mat& img_lig_now,
	int nSample)
{
	using Samples = cv::Mat[nSample];
	Samples horizontal_lines_pre, horizontal_lines_now;
	Samples vertical_lines_pre, vertical_lines_now;

	for (int i = 0; i < nSample; i++)
	{
		horizontal_lines_now[i] = img_lig_now.row(height * (i + 1) / (nSample + 1) - 1);
		horizontal_lines_pre[i] = img_lig_pre.row(height * (i + 1) / (nSample + 1) - 1);
		vertical_lines_now[i] = img_lig_now.col(width * (i + 1) / (nSample + 1) - 1);
		vertical_lines_pre[i] = img_lig_pre.col(width * (i + 1) / (nSample + 1) - 1);
	}

	double min_x_val{1e9}, min_y_val{1e9};
	int offset_x, offset_y;
	for (int offset = -5; offset <= 5; ++offset)
	{
		double val_x{ 0 }, val_y{ 0 };
		for (int i = 0; i < nSample; ++i)
		{
			cv::Mat a_x = horizontal_lines_pre[i].colRange(
				std::max(offset, 0), width + min(offset, 0));
			cv::Mat b_x = horizontal_lines_now[i].colRange(
				std::max(-offset, 0), width + min(-offset, 0));
			cv::Mat a_y = vertical_lines_pre[i].rowRange(
				std::max(offset, 0), height + min(offset, 0));
			cv::Mat b_y = vertical_lines_now[i].rowRange(
				std::max(-offset, 0), height + min(-offset, 0));
			val_x += cv::norm(a_x, b_x, NORM_L1);
			val_y += cv::norm(a_y, b_y, NORM_L1);
		}
		val_x /= nSample, val_y /= nSample;
		if (val_x < min_x_val)
			min_x_val = val_x, offset_x = offset;
		if (val_y < min_y_val)
			min_y_val = val_y, offset_y = offset;
	}

	return std::make_pair(offset_x, offset_y);
}

void 
ipt::IPT_Receiver::Preprocess(cv::Mat& img_sub)
{
	cv::normalize(img_sub, img_sub, 0, 255, NORM_MINMAX);
	img_sub.convertTo(img_sub, CV_8U);

	cv::Size bSize{ int(5 * scale_f_params), int(5 * scale_f_params) };
	cv::blur(img_sub, img_sub, bSize);

	int kSize = int(5 * scale_f_params) + 1 - int(5 * scale_f_params) % 2;
	double threshold = ipt::median_mat(img_sub, 256);
	cv::threshold(img_sub, img_sub, threshold, 255, cv::THRESH_BINARY);
	cv::medianBlur(img_sub, img_sub, kSize);

	cv::Size kernelSize{ int(6 * scale_f_params), int(6 * scale_f_params) };
	cv::Mat kernel{ cv::Mat::ones(kernelSize, CV_8UC1) };
	cv::morphologyEx(img_sub, img_sub, cv::MORPH_OPEN, kernel);
	kernelSize = cv::Size{ int(15 * scale_f_params), int(15 * scale_f_params) };
	kernel = cv::Mat::ones(kernelSize, CV_8UC1);
	cv::morphologyEx(img_sub, img_sub, cv::MORPH_CLOSE, kernel);
}

cv::Mat ipt::IPT_Receiver::Substraction(cv::Mat& img_lightness_1, cv::Mat& img_lightness_2)
{
	auto offset = AlignImg(img_lightness_1, img_lightness_2);
	cv::Mat warpMat = (cv::Mat_<float>(2, 3) << 1, 0, offset.first, 0, 1, offset.second);
	cv::warpAffine(
		img_lightness_2, img_lightness_2, warpMat, 
		cv::Size(width, height), INTER_NEAREST
	);

	cv::Mat sub = img_lightness_2 - img_lightness_1;

	if (offset.first)
	{
		sub.colRange(0, (int) abs(offset.first)).setTo(0);
		sub.colRange(width - (int) abs(offset.first), width).setTo(0);
	}
	if (offset.second)
	{
		sub.rowRange(0, (int) abs(offset.second)).setTo(0);
		sub.rowRange(height - (int)abs(offset.second), height).setTo(0);
	}

	return sub;
}

int ipt::IPT_Receiver::Detection(const cv::Mat& img, zarray_t *& detections)
{
	image_u8_t img_u8 = { 
		.width = img.cols, 
		.height = img.rows, 
		.stride = img.cols, 
		.buf = img.data };
	detections = apriltag_detector_detect(td, &img_u8);
	return zarray_size(detections);
}

ipt::IPT_Receiver::IPT_Receiver(
	const std::string& cam_path,
	const std::string& map_path,
	int w, int h, double scale_f)
{
	scale_factor = scale_f;
	width = w * scale_f;
	height = h * scale_f;
	scale_f_params = width / 1280.;

	for (int i = 0; i < 3; i++)
		img_org[i] = cv::Mat{ h, w, CV_8UC3 }, img_lightness[i] = cv::Mat{ h, w, CV_16SC1 };

	cam_mtx = cv::Mat(3, 3, CV_64F);
	cam_dist = cv::Mat(1, 5, CV_64F);
	ipt::read_cam_para(cam_path, cam_mtx, cam_dist);
	
	map_info = ipt::read_map_info(map_path);
	td = apriltag_detector_create();
	tf = tag36h11_create();
	apriltag_detector_add_family(td, tf);
}

ipt::IPT_Receiver::~IPT_Receiver()
{
	tag36h11_destroy(tf);
	apriltag_detector_destroy(td);
}

void ipt::IPT_Receiver::Demodulate(
	const cv::Mat& img_pre, 
	const cv::Mat& img_now, 
	const cv::Mat& img_nxt, 
	zarray_t*& detections)
{
	img_org[0] = img_pre;
	img_org[1] = img_now;
	img_org[2] = img_nxt;

	if (scale_factor != 1.0)
		for (int i = 0; i < 3; ++i)
			cv::resize(img_org[i], img_org[i], cv::Size(), 
				scale_factor, scale_factor, INTER_LINEAR);
	for (int i = 0; i < 3; i++)
		GetLightnessCh(img_org[i], img_lightness[i]);

	cv::Mat sub = Substraction(img_org[0], img_org[1]);
	Preprocess(sub);

	int ret = Detection(sub, detections);
	if (ret > 0)
	{
		tag_exist_flag = true;
		return;
	}
	sub = 255 - sub;
	ret = Detection(sub, detections);
	if (ret > 0)
	{
		tag_exist_flag = true;
		return;
	}

	sub = Substraction(img_org[1], img_org[2]);
	Preprocess(sub);
	ret = Detection(sub, detections);
	if (ret > 0)
	{
		tag_exist_flag = true;
		return;
	}
	sub = 255 - sub;
	ret = Detection(sub, detections);
	if (ret > 0)
	{
		tag_exist_flag = true;
		return;
	}
}