#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <memory.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <errno.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include "opencv4/opencv2/opencv.hpp"
#include "apriltag/apriltag.h"
#include "nlohmann/json.hpp"
#include "ipt_receiver.h"
#include "ipt_receiver_new.h"
#include "v4l2uvc.h"
#include "avilib.h"

#define PORT 65432
#define MAXLINE 1024
#define HOST_IP "192.168.50.216"

using namespace std;
using namespace cv;
using json = nlohmann::json;


int main() {
    /* parameters */
//    const string data_path = "/home/pi/UAV-Localization-based-on-the-QR-code/data_real/20210530_full_data/";
//    const string video_name = "0530_color_120fps_L=4_9x9_noLight_720p_with_optitrack.avi";
    const string data_path = "/home/pi/UAV-Localization-based-on-the-QR-code/data_real/20210905_data/";
    const string video_name = "640x360_2.avi";

    const string map_para_path = "./params/map_info_9x9.json";
    const string device = "/dev/video0";

    const string cam_para_path = "./params/cam_para_80d_1280x720.json";  // be careful to choose: /cam_para_80d_1280x720.json, /cam_para_90d_640x360.json
    // 最初的0530数据用的应该是/cam_para_80d_1280x720.json，飞机实际飞行用的应该是/cam_para_80d_640x360.json
    // 之前买过90d的，但是因为镜头变了，出现了严重的彩色闪烁，导致数据没法用。
    bool use_camera = false;
    double scale_f = 1.0;   // 0.5, 1.0
    int set_width = 640;   // 1280, 640
    int set_height = 360;   // 720, 360

    /* init */
    // UDP
    int sockfd;
    char buffer[MAXLINE];
    struct sockaddr_in servaddr;

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    inet_aton(HOST_IP, &servaddr.sin_addr);

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // opencv
    VideoCapture cap;

    if (use_camera) {
        cap = VideoCapture(device, CAP_V4L2);
    } else {
        cap = VideoCapture(data_path + video_name);
    }

    if (!cap.isOpened()) {
        cout << "Could not open reference " << device << endl;
        return -1;
    }

    int org_width;
    int org_height;
    int org_fps;

    if (use_camera) {
        org_width = set_width;
        org_height = set_height;
        org_fps = 120;

        cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(CAP_PROP_FRAME_WIDTH, org_width);
        cap.set(CAP_PROP_FRAME_HEIGHT, org_height);
        cap.set(CAP_PROP_FPS, org_fps);
    } else {
        org_width = cap.get(CAP_PROP_FRAME_WIDTH);
        org_height = cap.get(CAP_PROP_FRAME_HEIGHT);
        org_fps = cap.get(CAP_PROP_FPS);
    }


    ipt::IPTReceiver receiver(cam_para_path, map_para_path, org_width, org_height, scale_f, false);

    Mat f_pre(org_height, org_width, CV_8UC3);
    Mat f_now(org_height, org_width, CV_8UC3);
    Mat f_nxt(org_height, org_width, CV_8UC3);

    zarray_t *detections;
    Vec3d position;
    Vec3d angle;

    int cnt = 0;
    clock_t time_start = clock();

    cout << "initialization succeeded!" << endl;
    for (;;) {
        clock_t time_0 = clock();
        // get 3 successive frames

//        memset(&vd->buf, 0, sizeof(struct v4l2_buffer));

        cap >> f_pre;   // equal to cap.read(frame);
        if (f_pre.empty()) {
            cout << " <<<  Video end!  >>> " << endl;
            break;
        }

        cap >> f_now;   // equal to cap.read(frame);
        if (f_now.empty()) {
            cout << " <<<  Video end!  >>> " << endl;
            break;
        }

        cap >> f_nxt;   // equal to cap.read(frame);
        if (f_nxt.empty()) {
            cout << " <<<  Video end!  >>> " << endl;
            break;
        }

//        clock_t time_1 = clock();
        receiver.demodulate(f_pre, f_now, f_nxt, detections);
//        clock_t time_2 = clock();
//        cout << " <<< Time Counter for Demodulation: using " << (double) (time_2 - time_1) / CLOCKS_PER_SEC
//             << " seconds in total >>> "
//             << endl;

        receiver.estimate_pose(detections, position, angle);

        // cout information
        cout << "id = " << cnt << "; ";

        clock_t time_3 = clock();
        //cout << "time = " << (double) (time_3 - time_start) / CLOCKS_PER_SEC << "s; ";
        //cout << "speed = " << CLOCKS_PER_SEC / (double) (time_3 - time_0) << " fps; ";

        if (receiver.tag_exist_flag) {
            cout << "x = " << position[0] << "; "
                 << "y = " << position[1] << "; "
                 << "z = " << position[2] << "; "
                 << "roll = " << angle[0] * 180 / M_PI << "; "
                 << "pitch = " << angle[1] * 180 / M_PI << "; "
                 << "yaw = " << angle[2] * 180 / M_PI << "; ";

            // UDP
            
            /*
            json j;
            j["name"] = "uav_0";
            j["x"] = position[0];
            j["y"] = position[1];
            j["z"] = position[2];
            j["roll"] = angle[0];
            j["pitch"] = angle[1];
            j["yaw"] = angle[2];

            string msg_s = j.dump();
            char *msg = &msg_s[0];

            if (sendto(sockfd, (const char *) msg, strlen(msg), MSG_CONFIRM, (const struct sockaddr *) &servaddr,
                       sizeof(servaddr)) != -1) {
                cout << "Msg sent!";
            } else {
                cout << "UDP sent failed!";
            }*/
            
        }
        //cout << endl;

        cnt++;
//        if (cnt == 1000) {
//            break;
//        }

    }
    char msg[] = "quit";
    sendto(sockfd, (const char *) msg, strlen(msg), MSG_CONFIRM, (const struct sockaddr *) &servaddr,
           sizeof(servaddr));

    clock_t time_end = clock();
    cout << " <<< Time Counter in Total: using " << (double) (time_end - time_start) / CLOCKS_PER_SEC
         << " seconds in total >>> "
         << endl;

    close(sockfd);
    return 0;
}

