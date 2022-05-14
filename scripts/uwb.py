#!/usr/bin/env python3

import roslib
roslib.load_manifest("ipt_cpp")
import rospy as rp
import numpy as np
import serial
import time
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped

def uint16(LO, HI):
	return (LO & 0xFF) | (HI & 0xFF) << 8

def uint24(val0, val1, val2):
	if (val2 & 0xff) != 0:
		return ((val0 << 8 | val1 << 16 | val2 << 24) >> 8) - 0x1000000
	else:
		return (val0 << 8 | val1 << 16 | val2 << 24) >> 8

def recv(serial):
	while True:
		data = serial.read_all()
		if data == '':
			continue
		else:
			break
		time.sleep(0.01)
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

def unpack_bytes(recv_bytes, length, num_tag):
	position_list = np.zeros((num_tag,4))
	for i in range(num_tag):
		id_num = recv_bytes[28+20*i]
		position_list[i][0] = uint24(recv_bytes[28+20*i+1], recv_bytes[28+20*i+2], recv_bytes[28+20*i+3])
		position_list[i][1] = uint24(recv_bytes[28+20*i+4], recv_bytes[28+20*i+5], recv_bytes[28+20*i+6])
		position_list[i][2] = uint24(recv_bytes[28+20*i+7], recv_bytes[28+20*i+8], recv_bytes[28+20*i+9])
		position_list[i][3] = id_num
	return position_list

COVARIANCEARRAY = [
0.004, 0, 0, 0, 0, 0,
0, 0.004, 0, 0, 0, 0,
0, 0, 0.004, 0, 0, 0,
0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0,
]

if __name__ == '__main__':
	rp.init_node('uwb_node')
	rp.myargv(argv = sys.argv)

	num_tag = 1
	seq = 0
	position_list = np.zeros((num_tag,4))

	tagId = rp.get_param("~TagId", 0)
	topicName = rp.get_param("~TopicName", "uwb/pose_cov")
	serialName = rp.get_param("~SerialName", "/dev/ttyUSB0")
	baudrate = rp.get_param("~Baudrate", 921600)
	covariance = rp.get_param("~CovarianceArray", COVARIANCEARRAY)

	uwb_serial = serial.Serial(serialName, baudrate);
	if not uwb_serial.isOpen():
		rp.logfatal("Cannot open serial port")
		exit()

	pub = rp.Publisher(topicName, geometry_msgs.msg.PoseWithCovarianceStamped)
	while not rp.is_shutdown():
		data = recv(uwb_serial)
		for i in range(len(data) - 1):
			if data[i] == 0x55 and data[i+1] == 0x03:
				length_frame = uint16(data[i+2], data[i+3])
				recv_bytes = data[i:i+length_frame]
				if len(recv_bytes) == length_frame and length_frame >= (28 + num_tag * 20) and verifySum(recv_bytes, length_frame) :
					position_list = unpack_bytes(recv_bytes, length_frame, num_tag)

					foundTag = False
					tagIndex = -1
					for j in range(num_tag):
						if position_list[j][3] == tagId:
							foundTag = True
							tagIndex = j
							break
					if foundTag :
						Pose = geometry_msgs.msg.PoseWithCovarianceStamped()
						Pose.header.seq = seq
						seq += 1
						Pose.header.frame_id = "uwb"
						Pose.header.stamp = rp.get_rostime()
						Pose.pose.pose.x = position_list[tagIndex][0] / 1000
						Pose.pose.pose.y = position_list[tagIndex][1] / 1000
						Pose.pose.pose.z = position_list[tagIndex][2] / 1000
						Pose.pose.covariance = covariance
						pub.publish(Pose)
					else:
						rp.logwarn("Message received from serial port, but the tag was not found")
					break
		
