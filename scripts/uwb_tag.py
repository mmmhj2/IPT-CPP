#!/usr/bin/env python3
'''
Uses protocol NLink_LinkTrack_Tag_Frame0 
c.f. https://ftp.nooploop.com/software/products/uwb/doc/NLink_V1.3.pdf
'''

import roslib
roslib.load_manifest("ipt_cpp")
import rospy as rp
import numpy as np
import serial
import time
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped

COVARIANCEARRAY = [
0.006, 0, 0, 0, 0, 0,
0, 0.006, 0, 0, 0, 0,
0, 0, 0.006, 0, 0, 0,
0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0,
]

def uint16(LO, HI):
    return (LO & 0xFF) | (HI & 0xFF) << 8

def int24(LO, MI, HI):
    if (HI & 0xC0) != 0:
        return ((LO << 8 | MI << 16 | HI << 24) >> 8) - (1 << 24)
    else:
        return (LO << 8 | MI << 16 | HI << 24) >> 8

def uint24(LO, MI, HI):
    return (LO << 8 | MI << 16 | HI << 24) >> 8

def recv(serial):
    while True:
        data = serial.read_all()
        if len(data) <= 0:
            time.sleep(0.01)
            continue
        else:
            break
        
    return data

def verifySum(recv_bytes, length):
    byte_sum = 0
    for j in range(length - 1):
        byte_sum += recv_bytes[j]
    byte_sum = (byte_sum & 0xff)

    if byte_sum == recv_bytes[length-1]:
        return True
    else:
        return False

if __name__ == "__main__":
    rp.init_node('uwb_node')
    rp.myargv(argv = sys.argv)

    seq = 0
    rate = rp.Rate(60)
    tagId = rp.get_param("~TagId", 0)
    topicName = rp.get_param("~TopicName", "uwb/pose_cov")
    serialName = rp.get_param("~SerialName", "/dev/ttyUSB0")
    UWBFrame = rp.get_param("~UWBFrame", "uwb")
    baudrate = rp.get_param("~Baudrate", 115200)
    covariance = rp.get_param("~CovarianceArray", COVARIANCEARRAY)

    uwb_serial = serial.Serial(serialName, baudrate);
    if not uwb_serial.isOpen():
        rp.logfatal("Cannot open serial port")
        exit(-1)

    pub = rp.Publisher(topicName, PoseWithCovarianceStamped)
    while not rp.is_shutdown():
        rate.sleep()
        data = recv(uwb_serial)
        for i in range(len(data) - 1):
            if data[i] == 0x55 and data[i+1] == 0x01:
                if data[i+2] != tagId :
                    rp.logwarn("Inconsistent tag id, got %d, expecting %d", data[i+2], tagId)
                    continue
                if(len(data) - i < 128):
                    rp.logwarn("Got fragmented message")
                    continue
                frame = data[i:i+128]
                if not verifySum(frame, 128):
                    rp.logerror("Got corrupted message")
                    continue
                # Little-Endian
                x = int24(frame[4], frame[5], frame[6]) / 1000
                y = int24(frame[7], frame[8], frame[9]) / 1000
                z = int24(frame[10], frame[11], frame[12]) / 1000
                pose = PoseWithCovarianceStamped()
                pose.header.seq = seq
                seq += 1
                pose.header.frame_id = UWBFrame
                pose.header.stamp = rp.get_rostime()
                pose.pose.pose.x = x
                pose.pose.pose.y = y
                pose.pose.pose.z = z
                pose.pose.covariance = covariance
                pub.publish(pose)
