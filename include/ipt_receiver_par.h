/*
* Author: Li
* File: ipt_receiver_par.h
* Description: Refractored version of ipt::IPTReceiver, using C++11 concurrency support
*/
#ifndef IPT_RECEIVER_PAR_H
#define IPT_RECEIVER_PAR_H

#include "ipt_receiver_new.h"

namespace ipt
{
	class IPT_Receiver_Parallel : public IPT_Receiver
	{
	private:
		void CopyAndConvert(int index, cv::Mat from);
		// Parallel detection
		int Detection(const cv::Mat & img, zarray_t *& detections);

	public:
		IPT_Receiver_Parallel(const std::string& cam_path,
			const std::string& map_path,
			int w, int h,
			double scale_f = 1);
		virtual ~IPT_Receiver_Parallel();

		virtual void Demodulate(cv::Mat img_pre,
			cv::Mat img_now,
			cv::Mat img_nxt,
			zarray_t*& detections);
	};
}

#endif
