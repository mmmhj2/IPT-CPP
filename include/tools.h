//
// Created by lijin on 2021/7/29.
//

#ifndef IPT_CPP_TOOLS_H
#define IPT_CPP_TOOLS_H

#include "opencv4/opencv2/opencv.hpp"
#include "nlohmann/json.hpp"
#include <iostream>
#include <fstream>

using namespace cv;

struct TagInfo {
    int id;
    double size;
    double x, y, z;
    double qw, qx, qy, qz;
};

struct MapInfo {
    std::string name;
    std::string unit;
    int width_num, height_num;
    std::vector<struct TagInfo> layout;
};

namespace ipt {
    int read_cam_para(const std::string &cam_path, Mat &cam_mtx, Mat &cam_dist);

    struct MapInfo read_map_info(const std::string &map_path);

    double median_mat(cv::Mat Input, int nVals);

    Vec3d rotation_2_euler(Mat &R);
    Vec4d rotation_2_quaternion(Mat &R);
    Vec4d euler_2_quaternion(Vec3d& E);
    
    Mat quaternion_2_rotation(Vec4d& Q);
    Mat quaternion_2_rotation(double w, double x, double y, double z);
}

#endif //IPT_CPP_TOOLS_H
