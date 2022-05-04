//
// Created by lijin on 2021/7/29.
//

#ifndef IPT_CPP_TOOLS_H
#define IPT_CPP_TOOLS_H

#include "opencv4/opencv2/opencv.hpp"
#include "nlohmann/json.hpp"
#include <iostream>
#include <fstream>

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
    int read_cam_para(const std::string &cam_path, cv::Mat &cam_mtx, cv::Mat &cam_dist);

    struct MapInfo read_map_info(const std::string &map_path);

    double median_mat(cv::Mat Input, int nVals);

    cv::Vec3d rotation_2_euler(const cv::Mat &R);
    cv::Vec4d rotation_2_quaternion(const cv::Mat &R);
    cv::Vec4d euler_2_quaternion(const cv::Vec3d& E);
    
    cv::Mat quaternion_2_rotation(const cv::Vec4d& Q);
    cv::Mat quaternion_2_rotation(double w, double x, double y, double z);

    // Return a rotation matrix that turns a rotation in ENU frame to local frame
    // Asserting that the matrixENU is obtained when aligned to local frame
    cv::Mat AlignRotationMatrix(const cv::Mat& matrixENU);
    cv::Mat AlignRotationMatrix(const cv::Vec4d& quatENU);
}

#endif //IPT_CPP_TOOLS_H
