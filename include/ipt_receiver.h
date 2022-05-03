/*
Author: LI Jinjie
File: ipt_demodulator.py
Date: 2021/7/30 16:16
Description: file content
*/

#ifndef IPT_CPP_IPT_RECEIVER_H
#define IPT_CPP_IPT_RECEIVER_H

// apriltag related
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"

#include "tools.h"
#include "opencv4/opencv2/opencv.hpp"

#include <iostream>
#include <fstream>
#include <utility>
#include <pthread.h>
#include <semaphore.h>
#include <mutex>
#include <condition_variable>

//using namespace cv;

namespace ipt {

    class IPTReceiver {
    private:
        // camera params
        cv::Mat cam_mtx;
        cv::Mat cam_dist;

        // map info
        struct MapInfo map_info;

        // apriltag related
        apriltag_detector_t *td = apriltag_detector_create();
        apriltag_family_t *tf = tag36h11_create();

        // flags
        bool multi_threads_flag = true;

        [[noreturn]] static void *_thread_detect(void *args);

        // static: Static members of a class are not associated with the objects of the class.
        static void _get_lightness_ch(cv::Mat &img, cv::Mat &f_lightness_now);

        static void _align_img(cv::Mat &f_l_pre, cv::Mat &f_l_now, int &offset_x, int &offset_y, int N_sample = 3);

        static void _preprocess(cv::Mat &f_sub);

        void _lookup_map(int index_num, int idx, cv::Mat &obj_pts);

        void _demodulate_multi_t(cv::Mat &f_pre, cv::Mat &f_now, cv::Mat &f_nxt, zarray_t *&detections);

        void _demodulate_one_t(cv::Mat &f_pre, cv::Mat &f_now, cv::Mat &f_nxt, zarray_t *&detections);


    public:
        bool tag_exist_flag = false;  // set to false at the beginning of demodulate(), and to true if detection is successful

        IPTReceiver(const std::string &cam_path, const std::string &map_path,
                    int w, int h, double scale_f = 1., bool multi_threads = false);

        ~IPTReceiver();

        void demodulate(cv::Mat &f_pre, cv::Mat &f_now, cv::Mat &f_nxt, zarray_t *&detections);

        void estimate_pose(zarray_t *&detections, cv::Vec3d &position, cv::Vec3d &angle);

    };
}
#endif //IPT_CPP_IPT_RECEIVER_H

