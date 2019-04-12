#!/usr/bin/env python

import time, sys, os, math
import numpy as np
import h5py
from ros import rosbag
import roslib, rospy
from cv_bridge import CvBridge
import cv2
#roslib.load_manifest('sensor_msgs')
#from sensor_msgs.msg import Image

TOPIC = 'camera/image_raw'

filename = '/home/jean-michel/PythonScripts/test.mat' # filename
arrays = {}
f = h5py.File(filename)
print(f.items())
for k, v in f.items():
	print(k)
	arrays[k] = np.array(v)
	vid = np.array(v)
vid = np.squeeze(vid)
nFrame 	= vid.shape[0]
bagname = 'test_mat_v0.bag'
bag = rosbag.Bag(bagname, 'w')
frame_id = 0
prop_fps = 20
cb = CvBridge()
for kk in range(nFrame):
 	print(kk)
	frame = vid[kk,:,:]
	tranf_frame = frame.T
	stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
	frame_id += 1
	image = cb.cv2_to_imgmsg(tranf_frame, encoding='mono8')
	image.header.stamp = stamp
	image.header.frame_id = "camera"
	bag.write(TOPIC, image, stamp)

bag.close()
print("---------- Done ------------")
