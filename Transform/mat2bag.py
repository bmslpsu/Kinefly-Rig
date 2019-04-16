#!/usr/bin/env python
import glob, os
import time, sys, math
import numpy as np
import h5py
from ros import rosbag
import roslib, rospy
from cv_bridge import CvBridge
import cv2
from os import listdir
###################################################################################################
dispFlag = False
TOPIC = 'camera/image_raw' # topic
root = '/media/jean-michel/Seagate Backup Plus Drive/0/Vid'
os.chdir(root)
for file in glob.glob("*.mat"):
    filepath    = root + '/' + file
    filename    = os.path.splitext(os.path.basename(file))[0]
    bagname     = filename  + '.bag'       # output file .bag
    bagpath     = root + '/bag/' + bagname

    print('Converting:')
    print(filepath)

    f = h5py.File(filepath,'r') # load file
    arrays  = {}
    for k, v in f.items():
        #print(k)
        arrays[k] = np.array(v)

    vid     = arrays['vidData']     # video matrix
    vid     = np.squeeze(vid)       # get rid of singleton dimension
    nFrame  = vid.shape[0]          # # of frames
    tt      = arrays['t_v']         # time vector

    Fs      = np.round(1/np.mean(np.diff(tt)),decimals=0,out=None)/4 # frame rate
    print('FRAME RATE:')
    print(Fs)

    bag = rosbag.Bag(bagpath, 'w')
    cb = CvBridge()
    frame_id = 0
    for kk in range(nFrame):
		frame   = vid[frame_id,:,:].T	# get frame

		if dispFlag:
			print(frame_id)
			cv2.imshow('img',frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

		stamp 			= rospy.rostime.Time.from_sec(float(frame_id) / Fs)
		image              	= cb.cv2_to_imgmsg(frame, encoding='mono8')
		image.header.stamp 	= stamp
		frame_id 			+=1

		image.header.frame_id    = "camera"
		bag.write(TOPIC, image,stamp)

    bag.close()

    print('Done with:')
    print(bagpath)
    print('')
    if dispFlag:
        cv2.waitKey(0)

print("---------- DONE ------------")
