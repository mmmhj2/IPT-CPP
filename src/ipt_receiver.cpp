/*
Author: LI Jinjie
File: ipt_demodulator.py
Date: 2021/7/30 16:16
Description: find the location of tags,
*/

#include "ipt_receiver.h"


using namespace cv;
using namespace std;
using namespace ipt;

namespace ipt {
    // communication between threads
    pthread_mutex_t mtx_1 = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t mtx_2 = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t mtx_3 = PTHREAD_MUTEX_INITIALIZER;

    pthread_cond_t cond_var = PTHREAD_COND_INITIALIZER;

    bool ready_flag[] = {true, true, true, true};  // true: working  false: waiting

    // original frames
    Mat f_pre_org, f_now_org, f_nxt_org;

    // lightness channel
    Mat f_pre_l, f_now_l, f_nxt_l;

    // four images waiting for detection. 0 for main thread, 1,2,3 for thread 1,2,3
    Mat f_0, f_1, f_2, f_3;
    zarray_t *detections_0, *detections_1, *detections_2, *detections_3;

    // The child threads
    pthread_t tid_1, tid_2, tid_3;

    // paras that can be accessed by all threads
    int width;
    int height;
    double scale_factor;
    double scale_f_params;
}

// Constructor
IPTReceiver::IPTReceiver(const std::string &cam_path, const std::string &map_path, const int w, const int h,
                         const double scale_f, bool multi_threads) {
    multi_threads_flag = multi_threads;

    scale_factor = scale_f;
    width = w * scale_f; // scaled
    height = h * scale_f; // scaled
    scale_f_params = width / 1280.; // 720p is the standard resolution

    // initialize global vars
    f_pre_org = Mat(h, w, CV_8UC3);
    f_now_org = Mat(h, w, CV_8UC3);
    f_nxt_org = Mat(h, w, CV_8UC3);

    f_pre_l = Mat(height, width, CV_16SC1);
    f_now_l = Mat(height, width, CV_16SC1);
    f_nxt_l = Mat(height, width, CV_16SC1);

    f_0 = Mat(height, width, CV_8UC1);
    f_1 = Mat(height, width, CV_8UC1);
    f_2 = Mat(height, width, CV_8UC1);
    f_3 = Mat(height, width, CV_8UC1);

    // multi threads related
    int p_num_1 = 1;
    int p_num_2 = 2;
    int p_num_3 = 3;
    if (multi_threads_flag) {
        if (pthread_create(&tid_1, nullptr, _thread_detect, (void *) &p_num_1) == 0) {
            cout << "Create thread 1 successfully !" << endl;
        } else {
            cout << "Fail to create thread 1 !" << endl;
        }

        if (pthread_create(&tid_2, nullptr, _thread_detect, (void *) &p_num_2) == 0) {
            cout << "Create thread 2 successfully !" << endl;
        } else {
            cout << "Fail to create thread 2 !" << endl;
        }

        if (pthread_create(&tid_3, nullptr, _thread_detect, (void *) &p_num_3) == 0) {
            cout << "Create thread 3 successfully !" << endl;
        } else {
            cout << "Fail to create thread 3 !" << endl;
        }
    }

    // read camera params
    cam_mtx = Mat(3, 3, CV_64F);
    cam_dist = Mat(1, 5, CV_64F);
    ipt::read_cam_para(cam_path, cam_mtx, cam_dist);

    // read tags map information
    map_info = ipt::read_map_info(map_path);

    // apriltag detector for the main thread
    apriltag_detector_add_family(td, tf);
//    td->nthreads = 4;  // 4 core for pi 4b+
//    td->quad_decimate = 2; // downscaling
}

// Destructor
IPTReceiver::~IPTReceiver() {
    tag36h11_destroy(tf);
    apriltag_detector_destroy(td);
}


void IPTReceiver::demodulate(Mat &f_pre, Mat &f_now, Mat &f_nxt, zarray_t *&detections) {
    /* demodulate tags
     * the img points in detections are the results of processing the scaled image */
    tag_exist_flag = false;

    if (multi_threads_flag) {
        _demodulate_multi_t(f_pre, f_now, f_nxt, detections);
    } else {
        _demodulate_one_t(f_pre, f_now, f_nxt, detections);
    }
}

void IPTReceiver::_demodulate_multi_t(Mat &f_pre, Mat &f_now, Mat &f_nxt, zarray_t *&detections) {
    /* No increasing, since OpenCV has raised three extra threads. Just use one version. */

    //    clock_t time_0 = clock();
    f_pre_org = f_pre;
    f_now_org = f_now;
    f_nxt_org = f_nxt;

    while (ready_flag[1] or ready_flag[2] or ready_flag[3]) {
        usleep(3);   // make sure all threads are under waiting
    }

//    cout << "Broadcast!" << endl;
    ready_flag[1] = true;
    ready_flag[2] = true;
    ready_flag[3] = true;
    pthread_cond_broadcast(&cond_var);

    while (ready_flag[1] or ready_flag[2] or ready_flag[3])
        usleep(3);   // make sure all threads are under waiting

//    cout << "All three threads have finished!" << endl;

//        clock_t time_1 = clock();

    int offset_x, offset_y;
    /* image_pre and image_now */
    // 1) image alignment
    offset_x = 0;
    offset_y = 0;
    _align_img(f_pre_l, f_now_l, offset_x, offset_y);

    // 2) subtraction
    Mat f_sub_1 = f_now_l - f_pre_l;
    // remove the outer black border
    if (offset_x != 0) {
        f_sub_1.colRange(0, (int) abs(offset_x)).setTo(0);
        f_sub_1.colRange((int) (width - abs(offset_x)), width).setTo(0);
    }
    if (offset_y != 0) {
        f_sub_1.rowRange(0, (int) abs(offset_y)).setTo(0);
        f_sub_1.rowRange((int) (height - abs(offset_y)), height).setTo(0);
    }

    /* image_now and image_nxt */
    // 1) image alignment
    offset_x = 0;
    offset_y = 0;
    _align_img(f_now_l, f_nxt_l, offset_x, offset_y);

    // 2) subtraction
    Mat f_sub_2 = f_nxt_l - f_now_l;
    // remove the outer black border
    if (offset_x != 0) {
        f_sub_2.colRange(0, (int) abs(offset_x)).setTo(0);
        f_sub_2.colRange((int) (width - abs(offset_x)), width).setTo(0);
    }
    if (offset_y != 0) {
        f_sub_2.rowRange(0, (int) abs(offset_y)).setTo(0);
        f_sub_2.rowRange((int) (height - abs(offset_y)), height).setTo(0);
    }

//    clock_t time_3 = clock();

    // 3) img preprocessing, including normalization, filtering, thresholding and morphology
    _preprocess(f_sub_1);
    _preprocess(f_sub_2);

    f_0 = f_sub_1;
    f_1 = 255 - f_sub_1;
    f_2 = f_sub_2;
    f_3 = 255 - f_sub_2;

//    imshow("f_0", f_0);
//    imshow("f_1", f_1);
//    imshow("f_2", f_2);
//    imshow("f_3", f_3);
//    waitKey(0);

//    clock_t time_4 = clock();

    //    cout << "Broadcast!" << endl;
    ready_flag[1] = true;
    ready_flag[2] = true;
    ready_flag[3] = true;
    pthread_cond_broadcast(&cond_var);

    // 4) tag detection
    image_u8_t img_u8 = {
            .width = f_0.cols,
            .height = f_0.rows,
            .stride = f_0.cols,
            .buf = f_0.data
    };
    detections_0 = apriltag_detector_detect(td, &img_u8);

    while (ready_flag[1] or ready_flag[2] or ready_flag[3]) {
        usleep(3);   // make sure all threads are under waiting
    }

    // choose the final result
    tag_exist_flag = true;
    if (zarray_size(detections_0) != 0) {
        detections = detections_0;
    } else if (zarray_size(detections_1) != 0) {
        detections = detections_1;
    } else if (zarray_size(detections_2) != 0) {
        detections = detections_2;
    } else if (zarray_size(detections_3) != 0) {
        detections = detections_3;
    } else {
        tag_exist_flag = false;
    }

//    clock_t time_5 = clock();

//    cout << " <<< Time Counter for Color Space Conversion: using " << (double) (time_1 - time_0) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
//    cout << " <<< Time Counter for Image alignment: using " << (double) (time_2 - time_1) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
//    cout << " <<< Time Counter for Image alignment and Sub: using " << (double) (time_3 - time_1) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
//    cout << " <<< Time Counter for Preprocessing: using " << (double) (time_4 - time_3) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
//    cout << " <<< Time Counter for Detecting: using " << (double) (time_5 - time_4) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
}

void IPTReceiver::_demodulate_one_t(Mat &f_pre, Mat &f_now, Mat &f_nxt, zarray_t *&detections) {

    f_pre_org = f_pre;
    f_now_org = f_now;
    f_nxt_org = f_nxt;

    /* =============== for f_pre and f_now ================== */

    // 0) resize and get the lightness channel
    if (scale_factor != 1.) {
        resize(f_pre_org, f_pre_org, Size(), scale_factor, scale_factor, INTER_LINEAR);
        resize(f_now_org, f_now_org, Size(), scale_factor, scale_factor, INTER_LINEAR);
    }
    _get_lightness_ch(f_pre_org, f_pre_l);
    _get_lightness_ch(f_now_org, f_now_l);

    // 1) image alignment
    int offset_x = 0;
    int offset_y = 0;
    _align_img(f_pre_l, f_now_l, offset_x, offset_y);

    // 2) subtraction
    Mat f_sub_1 = f_now_l - f_pre_l;
    // remove the outer black border
    if (offset_x != 0) {
        f_sub_1.colRange(0, (int) abs(offset_x)).setTo(0);
        f_sub_1.colRange((int) (width - abs(offset_x)), width).setTo(0);
    }
    if (offset_y != 0) {
        f_sub_1.rowRange(0, (int) abs(offset_y)).setTo(0);
        f_sub_1.rowRange((int) (height - abs(offset_y)), height).setTo(0);
    }

    // 3) img preprocessing, including normalization, filtering, thresholding and morphology
    _preprocess(f_sub_1);

    f_0 = f_sub_1;
    // 4) tag detection
    image_u8_t img_u8 = {
            .width = f_0.cols,
            .height = f_0.rows,
            .stride = f_0.cols,
            .buf = f_0.data
    };
    detections = apriltag_detector_detect(td, &img_u8);
    if (zarray_size(detections) != 0) {
        tag_exist_flag = true;
        return;
    }

    f_1 = 255 - f_sub_1;
    // 4) tag detection
    img_u8.buf = f_1.data;
    detections = apriltag_detector_detect(td, &img_u8);
    if (zarray_size(detections) != 0) {
        tag_exist_flag = true;
        return;
    }

    /* =============== for f_now and f_nxt ================== */
    // 0) resize and get the lightness channel
    if (scale_factor != 1.) {
        resize(f_nxt_org, f_nxt_org, Size(), scale_factor, scale_factor, INTER_LINEAR);
    }
    _get_lightness_ch(f_nxt_org, f_nxt_l);

    // 1) image alignment
    offset_x = 0;
    offset_y = 0;
    _align_img(f_now_l, f_nxt_l, offset_x, offset_y);

    // 2) subtraction
    Mat f_sub_2 = f_nxt_l - f_now_l;
    // remove the outer black border
    if (offset_x != 0) {
        f_sub_2.colRange(0, (int) abs(offset_x)).setTo(0);
        f_sub_2.colRange((int) (width - abs(offset_x)), width).setTo(0);
    }
    if (offset_y != 0) {
        f_sub_2.rowRange(0, (int) abs(offset_y)).setTo(0);
        f_sub_2.rowRange((int) (height - abs(offset_y)), height).setTo(0);
    }

    // 3) img preprocessing, including normalization, filtering, thresholding and morphology
    _preprocess(f_sub_2);

    f_2 = f_sub_2;
    // 4) tag detection
    img_u8.buf = f_2.data;
    detections = apriltag_detector_detect(td, &img_u8);
    if (zarray_size(detections) != 0) {
        tag_exist_flag = true;
        return;
    }

    f_3 = 255 - f_sub_2;
    // 4) tag detection
    img_u8.buf = f_3.data;
    detections = apriltag_detector_detect(td, &img_u8);
    if (zarray_size(detections) != 0) {
        tag_exist_flag = true;
        return;
    }

}

void IPTReceiver::_preprocess(Mat &img_sub) {

//    clock_t time_0 = clock();
    // 1) normalization and convert data type to CV_8U
    cv::normalize(img_sub, img_sub, 0, 255, NORM_MINMAX);
    img_sub.convertTo(img_sub, CV_8U);

//    clock_t time_1 = clock();

    // 2) filtering
    cv::blur(img_sub, img_sub, Size(int(5 * scale_f_params), int(5 * scale_f_params)));

//    clock_t time_2 = clock();

    // 3) thresholding
    double thresh = ipt::median_mat(img_sub, 256);  // 8 bit, so 2^8 = 256
    cv::threshold(img_sub, img_sub, thresh, 255, cv::THRESH_BINARY);  // extremely critical
    cv::medianBlur(img_sub, img_sub, int(5 * scale_f_params) + 1 - int(5 * scale_f_params) % 2);  // an odd number

//    clock_t time_3 = clock();

    // 4) morphology
    // Morphology open, to remove some noise.
    Mat kernel = Mat::ones(int(6 * scale_f_params), int(6 * scale_f_params), CV_8UC1);
    cv::morphologyEx(img_sub, img_sub, cv::MORPH_OPEN, kernel);
    kernel = Mat::ones(int(15 * scale_f_params), int(15 * scale_f_params), CV_8UC1);
    cv::morphologyEx(img_sub, img_sub, cv::MORPH_CLOSE, kernel);

//    clock_t time_4 = clock();
//    cout << " <<< Time Counter for Normalization: using " << (double) (time_1 - time_0) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
//    cout << " <<< Time Counter for Filtering: using " << (double) (time_2 - time_1) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
//    cout << " <<< Time Counter for Thresholding: using " << (double) (time_3 - time_2) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
//    cout << " <<< Time Counter for Morphology: using " << (double) (time_4 - time_3) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;

}

void IPTReceiver::_align_img(Mat &f_l_pre, Mat &f_l_now, int &offset_x, int &offset_y, int N_sample) {
    /* choose 3 horizontal lines and 3 vertical lines to get the best x,y bias, return aligned frame_now */

//    clock_t time_1 = clock();

    // horizontal lines
    Mat h_lines_pre[N_sample];
    for (int i = 0; i < N_sample; i++)
        h_lines_pre[i] = f_l_pre.row(height * (i + 1) / (N_sample + 1) - 1);

    Mat h_lines_now[N_sample];
    for (int i = 0; i < N_sample; i++)
        h_lines_now[i] = f_l_now.row(height * (i + 1) / (N_sample + 1) - 1);

    // vertical lines
    Mat v_lines_pre[N_sample];
    for (int i = 0; i < N_sample; i++)
        v_lines_pre[i] = f_l_pre.col(width * (i + 1) / (N_sample + 1) - 1);

    Mat v_lines_now[N_sample];
    for (int i = 0; i < N_sample; i++)
        v_lines_now[i] = f_l_now.col(width * (i + 1) / (N_sample + 1) - 1);

//    clock_t time_2 = clock();

    // calculate the offset
    double min_x_val = 99999;
    double min_y_val = 99999;

    double val_x, val_y;
    for (int offset = -5; offset <= 5; offset++) {
        // horizontal
        val_x = 0;
        for (int i = 0; i < N_sample; i++) {
            Mat a_x = h_lines_pre[i].colRange(max(offset, 0), width + min(offset, 0));
            Mat b_x = h_lines_now[i].colRange(max(-offset, 0), width + min(-offset, 0));
            val_x += cv::norm(a_x, b_x, NORM_L1);
        }
        val_x /= N_sample;  // mean
        if (val_x < min_x_val) {
            min_x_val = val_x;
            offset_x = offset;
        }

        // vertical
        val_y = 0;
        for (int i = 0; i < N_sample; i++) {
            Mat a_y = v_lines_pre[i].rowRange(max(offset, 0), height + min(offset, 0));
            Mat b_y = v_lines_now[i].rowRange(max(-offset, 0), height + min(-offset, 0));
            val_y += cv::norm(a_y, b_y, NORM_L1);
        }
        val_y /= N_sample;  // mean
        if (val_y < min_y_val) {
            min_y_val = val_y;
            offset_y = offset;
        }
    }

//    cout << "The final offset_x is:" << offset_x << endl;
//    cout << "The final offset_y is:" << offset_y << endl;

//    clock_t time_3 = clock();

    cv::Mat M = (cv::Mat_<float>(2, 3) << 1, 0, offset_x, 0, 1, offset_y);
    cv::warpAffine(f_l_now, f_l_now, M, cv::Size(width, height), INTER_NEAREST);

//    clock_t time_4 = clock();

//    cout << " <<< Time Counter for Getting lines: using " << (double) (time_2 - time_1) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
//    cout << " <<< Time Counter for Finding offset: using " << (double) (time_3 - time_2) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;
//    cout << " <<< Time Counter for warpAffine: using " << (double) (time_4 - time_3) / CLOCKS_PER_SEC
//         << " seconds in total >>> "
//         << endl;

}

void IPTReceiver::_get_lightness_ch(Mat &img, Mat &f_lightness_now) {

    cvtColor(img, img, COLOR_BGR2Lab);

    // get lightness channel and convert to int16 type
    vector<Mat> channels(3);
    split(img, channels);  // L, a, b
    channels[0].convertTo(f_lightness_now, CV_16SC1);
}

void IPTReceiver::estimate_pose(zarray_t *&detections, Vec3d &position, Vec3d &angle) {
    /* euler angle in radian: roll, pitch, yaw */
    if (!tag_exist_flag) {
        return;  // No tag is detected.
    }

    Mat R_w_c, R_c_w, rvec, tvec;
    // use all points for pnp solving
    Mat obj_pts = Mat::zeros(4 * zarray_size(detections), 3, CV_64FC1);
    Mat img_pts = Mat::zeros(4 * zarray_size(detections), 2, CV_64FC1);
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        // Do stuff with detections here.

        int index_num = 4 * i;
        // 3D object points
        _lookup_map(index_num, det->id, obj_pts);

        // 2D image points
        // from counter-clockwise to clockwise to meet IPPE_SQUARE's requirement
        for (int j = 0; j < 4; j++) {
            img_pts.at<double>(index_num + j, 0) = det->p[3 - j][0] / scale_factor;
            img_pts.at<double>(index_num + j, 1) = det->p[3 - j][1] / scale_factor;
        }

        cv::solvePnP(obj_pts, img_pts, cam_mtx, cam_dist, rvec, tvec, false, SOLVEPNP_IPPE);

        // PAY ATTENTION: rvec and tvec are world coordinates in camera coordinates
        // coordinates transform
        cv::Rodrigues(rvec, R_w_c);
        cv::transpose(R_w_c, R_c_w);

        Mat p = - R_c_w * tvec;

        // prepare data
        position[0] = p.at<double>(0,0);
        position[1] = p.at<double>(1,0);
        position[2] = p.at<double>(2,0);

        angle = rotation_2_euler(R_c_w);
    }
}

void IPTReceiver::_lookup_map(int index_num, int idx, Mat &obj_pts) {
    double bias = map_info.layout[idx].size / 2.;
    double x = map_info.layout[idx].x;
    double y = map_info.layout[idx].y;

    // this order is consistent with the required order of IPPE_SQUARE method.
    obj_pts.at<double>(0 + index_num, 0) = x - bias;
    obj_pts.at<double>(0 + index_num, 1) = y + bias;

    obj_pts.at<double>(1 + index_num, 0) = x + bias;
    obj_pts.at<double>(1 + index_num, 1) = y + bias;

    obj_pts.at<double>(2 + index_num, 0) = x + bias;
    obj_pts.at<double>(2 + index_num, 1) = y - bias;

    obj_pts.at<double>(3 + index_num, 0) = x - bias;
    obj_pts.at<double>(3 + index_num, 1) = y - bias;
}

[[noreturn]] void *IPTReceiver::_thread_detect(void *args) {
    int p_num = *(int *) args;

    Mat *f_org;
    Mat *f_l;
    Mat *f_bw;
    zarray_t **detections_ptr;
    pthread_mutex_t *mtx;

    switch (p_num) {
        case 1:
            f_org = &f_pre_org;  // 浅拷贝
            f_l = &f_pre_l;
            f_bw = &f_1;
            detections_ptr = &detections_1;
            mtx = &mtx_1;
            break;
        case 2:
            f_org = &f_now_org;  // 浅拷贝
            f_l = &f_now_l;
            f_bw = &f_2;
            detections_ptr = &detections_2;
            mtx = &mtx_2;
            break;
        case 3:
            f_org = &f_nxt_org;  // 浅拷贝
            f_l = &f_nxt_l;
            f_bw = &f_3;
            detections_ptr = &detections_3;
            mtx = &mtx_3;
            break;
        default:
            cout << "Error, no arguments are passed in!" << endl;
            break;
    }

    // apriltag related
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    for (;;) {
//        cout << "Thread " << p_num << " goes to sleep!" << endl;
        pthread_mutex_lock(mtx);
        ready_flag[p_num] = false;
        while (!ready_flag[p_num]) {
            pthread_cond_wait(&cond_var, mtx);
        }
        pthread_mutex_unlock(mtx);
//        cout << "Thread " << p_num << " wakeup!" << endl;

        if (scale_factor != 1.) {
            resize(*f_org, *f_org, Size(), scale_factor, scale_factor, INTER_LINEAR);
        }
        _get_lightness_ch(*f_org, *f_l);
//        cout << "Thread " << p_num << " finish cvtColot()!" << endl;

//        cout << "Thread " << p_num << " goes to sleep again!" << endl;
        pthread_mutex_lock(mtx);
        ready_flag[p_num] = false;
        while (!ready_flag[p_num]) {
            pthread_cond_wait(&cond_var, mtx);
        }
        pthread_mutex_unlock(mtx);
//        cout << "Thread " << p_num << " wakeup again !" << endl;

        // 4) tag detection
        image_u8_t img_u8 = {
                .width = (*f_bw).cols,
                .height = (*f_bw).rows,
                .stride = (*f_bw).cols,
                .buf = (*f_bw).data
        };
        *detections_ptr = apriltag_detector_detect(td, &img_u8);
//        cout << "Thread " << p_num << " finish tag detection !" << endl;
    }

}
