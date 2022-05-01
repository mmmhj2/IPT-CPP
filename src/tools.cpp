//
// Created by lijin on 2021/7/29.
//
#include "tools.h"

using namespace cv;
using namespace std;
using json = nlohmann::json;

namespace ipt {

    void read_cam_para(const std::string &cam_path, Mat &cam_mtx, Mat &cam_dist) {
        // Read camera params from a json file.
        // Method 1: using nlohmann::json library.
        std::ifstream in_file(cam_path);
        json cam_para;
        in_file >> cam_para;
        in_file.close();
        // Method 2: using OpenCV. Also need FleStorage to save data, thus, not suitable here.
        // Convert to cv::Mat format
//        Mat cam_mtx(3, 3, CV_32F), cam_dist(1, 5, CV_32F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cam_mtx.at<double>(i, j) = cam_para["mtx"][i][j];
            }
        }
        for (int i = 0; i < 1; i++) {
            for (int j = 0; j < 5; j++) {
                cam_dist.at<double>(i, j) = cam_para["dist"][i][j];
            }
        }
    }

    struct MapInfo read_map_info(const std::string &map_path) {
        std::ifstream in_file(map_path);
        if (!in_file.is_open()) {
            cout << "Fail to open the map_info file." << endl;
        }
        json map_info_j;
        in_file >> map_info_j;
        in_file.close();

        struct MapInfo map_info;

        map_info.name = map_info_j["name"];
        map_info.unit = map_info_j["unit"];
        map_info.width_num = map_info_j["width_num"];
        map_info.height_num = map_info_j["height_num"];

        for (int i = 0; i < map_info.width_num * map_info.height_num; i++) {
            struct TagInfo tag_info{};
            tag_info.id = map_info_j["layout"][i]["id"];
            tag_info.size = map_info_j["layout"][i]["size"];
            tag_info.x = map_info_j["layout"][i]["x"];
            tag_info.y = map_info_j["layout"][i]["y"];
            tag_info.z = map_info_j["layout"][i]["z"];
            tag_info.qw = map_info_j["layout"][i]["qw"];
            tag_info.qx = map_info_j["layout"][i]["qx"];
            tag_info.qy = map_info_j["layout"][i]["qy"];
            tag_info.qz = map_info_j["layout"][i]["qz"];

            map_info.layout.push_back(tag_info);
        }

        return map_info;
    }

    double median_mat(cv::Mat Input, int nVals) {
        /* from: https://stackoverflow.com/questions/30078756/super-fast-median-of-matrix-in-opencv-as-fast-as-matlab/30079823*/

        // COMPUTE HISTOGRAM OF SINGLE CHANNEL MATRIX
        float range[] = {0, static_cast<float>(nVals)};
        const float *histRange = {range};
        bool uniform = true;
        bool accumulate = false;
        cv::Mat hist;
        calcHist(&Input, 1, 0, cv::Mat(), hist, 1, &nVals, &histRange, uniform, accumulate);

        // COMPUTE CUMULATIVE DISTRIBUTION FUNCTION (CDF)
        cv::Mat cdf;
        hist.copyTo(cdf);
        for (int i = 1; i <= nVals - 1; i++) {
            cdf.at<float>(i) += cdf.at<float>(i - 1);
        }
        cdf /= Input.total();

        // COMPUTE MEDIAN
        double medianVal;
        for (int i = 0; i <= nVals - 1; i++) {
            if (cdf.at<float>(i) >= 0.5) {
                medianVal = i;
                break;
            }
        }
        return medianVal;
    }

    /* from the website: https://learnopencv.com/rotation-matrix-to-euler-angles/ */
    // Checks if a matrix is a valid rotation matrix.
    bool isRotationMatrix(Mat &R) {
        Mat Rt;
        transpose(R, Rt);
        Mat shouldBeIdentity = Rt * R;
        Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

        return norm(I, shouldBeIdentity) < 1e-6;
    }

    // Calculates rotation matrix to euler angles in Z-Y-X order.
    Vec3d rotation_2_euler(Mat &R) {

//        assert(isRotationMatrix(R));

        double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

        bool singular = sy < 1e-6; // If

        double roll, pitch, yaw;
        if (!singular) {
            roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
            pitch = atan2(-R.at<double>(2, 0), sy);
            yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        } else {
            roll = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
            pitch = atan2(-R.at<double>(2, 0), sy);
            yaw = 0;
        }
        return Vec3d(roll, pitch, yaw);
    }


    // Calculates rotation matrix to quaternion
    // from the book "Small Unmanned Aircraft Theory and Practice"
    Vec4d rotation_2_quaternion(Mat &R) {

//        assert(isRotationMatrix(R));

        double r11 = R.at<double>(0, 0);
        double r12 = R.at<double>(0, 1);
        double r13 = R.at<double>(0, 2);
        double r21 = R.at<double>(1, 0);
        double r22 = R.at<double>(1, 1);
        double r23 = R.at<double>(1, 2);
        double r31 = R.at<double>(2, 0);
        double r32 = R.at<double>(2, 1);
        double r33 = R.at<double>(2, 2);

        double tmp = r11 + r22 + r33;

        double e0, e1, e2, e3;
        if (tmp > 0) {
            e0 = 0.5 * sqrt(1 + tmp);
        } else {
            e0 = 0.5 * sqrt((pow(r12 - r21, 2) + pow(r13 - r31, 2) + pow(r23 - r32, 2)) / (3 - tmp));
        }

        tmp = r11 - r22 - r33;
        if (tmp > 0) {
            e1 = 0.5 * sqrt(1 + tmp);
        } else {
            e1 = 0.5 * sqrt((pow(r12 + r21, 2) + pow(r13 + r31, 2) + pow(r23 - r32, 2)) / (3 - tmp));
        }

        tmp = -r11 + r22 - r33;
        if (tmp > 0) {
            e2 = 0.5 * sqrt(1 + tmp);
        } else {
            e2 = 0.5 * sqrt((pow(r12 + r21, 2) + pow(r13 + r31, 2) + pow(r23 + r32, 2)) / (3 - tmp));
        }

        tmp = -r11 + -22 + r33;
        if (tmp > 0) {
            e3 = 0.5 * sqrt(1 + tmp);
        } else {
            e3 = 0.5 * sqrt((pow(r12 - r21, 2) + pow(r13 + r31, 2) + pow(r23 + r32, 2)) / (3 - tmp));
        }

        return Vec4d(e0, e1, e2, e3);
    }
    Vec4d euler_2_quaternion(Vec3d& E)
    {
        const double& roll = E[0];
        const double& pitch = E[1];
        const double& yaw = E[2];
        double w, x, y, z;
        x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
        z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
        w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        return Vec4d(w, x, y, z);
    }
}
