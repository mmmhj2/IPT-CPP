
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <string.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <X11/Xlib.h>
#include "v4l2uvc.h"
#include "avilib.h"

int main(int argc, char *argv[])   // -s 1280x720   -d  /dev/video1   -a  test.avi  -i 120  -t 5
{
    FILE *file = NULL;
    struct vdIn *vd;
    unsigned char *tmpbuffer;
    char *filename = NULL;
    char *avifilename = NULL;

    char *sizestring = NULL;
    char *fpsstring = NULL;
    char *timestring = NULL;
    char *separateur = NULL;
    int width;
    int height;
    int fps;
    int i;
    int time_total;

    time_t timer;
    struct tm *Now;

    // record time
    FILE *txt_file;
    char *txt_filename = "time.txt";

    // by default
    width = 640;
    height = 480;
    fps = 120;
    filename = "/dev/video0";
    avifilename = "test.avi";

    if (1 == argc) {
        printf("No parameters are detected. \n");
        width = 640;
        height = 480;
        fps = 120;
        filename = "/dev/video0";
        avifilename = "test.avi";
    } else {
        printf("The number of parameters is %d.\r\n", argc);
        for (i = 0; i < argc; i++) {

            if (argv[i] == NULL || *argv[i] == 0 || *argv[i] != '-')
                continue;

            if (strcmp(argv[i], "-s") == 0) {
                if (i + 1 >= argc) {
                    printf("No parameter specified with -s, aborting.\n");
                    exit(1);
                } else {
                    sizestring = strdup(argv[i + 1]);
                    width = strtoul(sizestring, &separateur, 10);
                    if (*separateur != 'x') {
                        printf("Error in size use -s widthxheight\n");
                        exit(1);
                    } else {
                        ++separateur;
                        height = strtoul(separateur, &separateur, 10);
                        if (*separateur != 0)
                            printf("hmm.. dont like that!! trying this height\n");
                    }
                    printf("width = %d, height = %d \n", width, height);
                }
            }

            if (strcmp(argv[i], "-d") == 0) {
                if (i + 1 >= argc) {
                    printf("No parameter specified with -d, aborting.\n");
                    exit(1);
                } else {
                    filename = strdup(argv[i + 1]);
                    printf("device name : %s \n", filename);
                }
            }

            if (strcmp(argv[i], "-t") == 0) {
                if (i + 1 >= argc) {
                    printf("No parameter specified with -t, aborting.\n");
                    exit(1);
                } else {
                    timestring = argv[i + 1];
                    time_total = strtoul(timestring, &separateur, 10);
                    if (*separateur != '\0') {
                        printf("Invalid frame rate '%s' specified with -i. "
                               "Only integers are supported. Aborting.\n", fpsstring);
                        exit(1);
                    }
                    printf("time : %d s \r\n", time_total);
                }
            }

            if (strcmp(argv[i], "-a") == 0) {
                if (i + 1 >= argc) {
                    printf("No parameter specified with -a, aborting.\n");
                    exit(1);
                } else {
                    avifilename = strdup(argv[i + 1]);
                    printf("AVI name : %s\n", avifilename);
                }
            }

            if (strcmp(argv[i], "-i") == 0) {
                if (i + 1 >= argc) {
                    printf("No parameter specified with -i, aborting.\n");
                    exit(1);
                } else {
                    fpsstring = argv[i + 1];
                    fps = strtoul(fpsstring, &separateur, 10);
                    if (*separateur != '\0') {
                        printf("Invalid frame rate '%s' specified with -i. "
                               "Only integers are supported. Aborting.\n", fpsstring);
                        exit(1);
                    }
                    printf("FPS : %d \n", fps);
                }
            }
        }
    }

    if (filename == NULL || *filename == 0) {
        filename = "/dev/video0";
    }

    if (avifilename == NULL || *avifilename == 0) {
        avifilename = "test.avi";
    }

    int format = V4L2_PIX_FMT_MJPEG;
    int ret;
    int grabmethod = 1;

    file = fopen(filename, "wb");
    if (file == NULL) {
        printf("Unable to open file for raw frame capturing\n ");
        exit(1);
    }

//v4l2 init
    vd = (struct vdIn *) calloc(1, sizeof(struct vdIn));
    if (init_videoIn(vd, (char *) filename, width, height, fps, format, grabmethod, avifilename) < 0) {
        exit(1);
    }

    if (video_enable(vd)) {
        exit(1);
    }

    vd->avifile = AVI_open_output_file(vd->avifilename);
    if (vd->avifile == NULL) {
        printf("Error opening avifile %s\n", vd->avifilename);
        exit(1);
    }

    AVI_set_video(vd->avifile, vd->width, vd->height, fps, "MJPG");
    printf("recording to %s\n", vd->avifilename);

    time(&timer);
    Now = localtime(&timer);

    txt_file = fopen(txt_filename, "w");
    if (txt_file != NULL) {
        fprintf(txt_file, "Hello world! \n");
        fprintf(txt_file, "%s\n", asctime(Now));
        fclose(txt_file);
    }

    while (vd->framecount < (time_total * fps)) {
        memset(&vd->buf, 0, sizeof(struct v4l2_buffer));
        vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vd->buf.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(vd->fd, VIDIOC_DQBUF, &vd->buf);
        if (ret < 0) {
            printf("Unable to dequeue buffer");
            exit(1);
        }

        memcpy(vd->tmpbuffer, vd->mem[vd->buf.index], vd->buf.bytesused);

        AVI_write_frame(vd->avifile, vd->tmpbuffer, vd->buf.bytesused, vd->framecount);

        vd->framecount++;

        ret = ioctl(vd->fd, VIDIOC_QBUF, &vd->buf);
        if (ret < 0) {
            printf("Unable to requeue buffer");
            exit(1);
        }
    }
    fclose(file);
    close_v4l2(vd);
}
