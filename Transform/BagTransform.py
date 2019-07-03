import glob, os
import numpy as np
import h5py
from ros import rosbag
import rospy
from cv_bridge import CvBridge
import cv2

class BagTransform:
	def __init__(self):
		
		# File name data
		self.name 		= rospy.get_name().rstrip('/')
		self.debug 		= False # default is off
		self.root 		= ''
		self.bagroot 	= ''
		self.file_all 	= ''
		self.n_file 	= 0
		self.file_path 	= ''
		self.file_root	= ''
		self.file_name 	= ''
		self.file_ext 	= ''
		self.bag_path 	= ''
		self.bag_root 	= ''
		self.fullpath	= {}
		
		# File content data
		self.vid_name	= ''
		self.time_name	= ''
		self.data		= {}
		self.vid		= np.zeros((1,1))
		self.time 		= np.zeros((1,1))
		self.n_frame	= 0
		self.fs			= 50 # default
		
		# Bag data
		self.topic		= 'camera/image_raw'
		self.bag		= ''
	
	def get_file_data(self, filepath):
	# Extract root, name, and extension from file path; make bag file path
		self.file_path 	= filepath
		self.file_root 	= os.path.dirname(self.file_path)
		self.file_name 	= os.path.basename(self.file_path)
		self.file_name 	= os.path.splitext(self.file_name)[0]
		
		self.bag_root 	= self.file_root + '/bag/'
		self.bag_path 	= self.bag_root + self.file_name + '.bag'
		
		if not os.path.isdir(self.bag_root):
			os.mkdir(self.bag_root, 0755)
		
	def get_files(self,root,ext):
	# Get all files in directory
		self.root = root
		self.bag_root = self.root + '/bag/'
		os.chdir(self.root)
		
		if os.path.isdir(self.bag_root):
			print('.bag directory exists')
			print "   ", self.bag_root
		else:
			print('Making .bag directory: ')
			print(self.bag_root)
			os.mkdir(self.bag_root, 0755)
		
		self.file_all = glob.glob(ext)
		self.file_all.sort()
		self.n_file = len(self.file_all)
	
		print('')
		print "Total Files: " , self.n_file
		print('')
			
	def get_matdata(self):
	# Extract video & time data from .mat file
		self.data = h5py.File(self.file_path,'r') # load file
		arrays = {}
		for k, v in self.data.items():
			arrays[k] = np.array(v)
		
		self.vid 		= arrays[self.vid_name]  	# video matrix
		self.vid 		= np.squeeze(self.vid)  	# get rid of singleton dimension
		self.n_frame 	= self.vid.shape[0]  		# # of frames
		
		try:
			self.time 		= arrays[self.time_name]  	# time vector
			self.fs 		= np.round(1/np.mean(np.diff(self.time)),decimals=0,out=None) # frame rate
		except:
			print "No time vector speicifed: play back at" , self.fs , "Hz"
		
	def mat2bag(self,filepath,vid_name,time_name,debug):
	# Write video matrix to ROS topic in .bag file
		self.vid_name 	= vid_name
		self.time_name 	= time_name
		self.debug 		= debug
		self.get_file_data(filepath)
		self.get_matdata()
		
		print "Converting:", self.file_path
		
		self.bag = rosbag.Bag(self.bag_path, 'w')
		cb = CvBridge()
		frame_id = 0
		for frame in range(self.n_frame):
			frame = self.vid[frame_id, :, :].T  # get frame
			
			if self.debug:
				#print(frame_id)
				cv2.imshow(self.file_name, frame)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break
			
			stamp = rospy.rostime.Time.from_sec(float(frame_id) / self.fs)
			image = cb.cv2_to_imgmsg(frame, encoding='mono8')
			image.header.stamp = stamp
			frame_id += 1
			
			image.header.frame_id = "camera"
			self.bag.write(self.topic, image, stamp)
		
		self.bag.close()
		
		cv2.destroyAllWindows()
		#if self.debug:
			#cv2.waitKey(0)
	
		print "Finished:  ", self.bag_path
		print('')
	
	def vid2bag(self, filepath, debug):
	# Write video to ROS topic in .bag file
		self.debug = debug
		self.get_file_data(filepath)
		
		print "Converting:", self.bag_path
		
		self.bag = rosbag.Bag(self.bag_path, 'w')
		cap = cv2.VideoCapture(self.file_path)
		cb = CvBridge()
		
		prop_fps = cap.get(cv2.CAP_PROP_FPS)
		if prop_fps != prop_fps or prop_fps <= 1e-2:
			print "Warning: can't get FPS. Assuming 24."
			prop_fps = self.fs
		
		self.fs = prop_fps
	
		ret = True
		frame_id = 0
		while ret:
			ret, frame = cap.read()
			if not ret:
				break
				
			if self.debug:
				# print(frame_id)
				cv2.imshow(self.file_name, frame)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break

			stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
			frame_id += 1
			image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
			image.header.stamp = stamp
			image.header.frame_id = "camera"
			self.bag.write(self.topic, image, stamp)
		cap.release()
		
		self.bag.close()
		cv2.destroyAllWindows()
		
		print "Finished:  ", self.bag_path
		print('')
		
	def batch_mat(self,rootdir,vid_name,time_name,debug):
	# For folder of .mat files
		self.get_files(rootdir,'*.mat') # only .mat files
		self.fullpath = {}
		for fIdx in range(self.n_file):
			self.fullpath[fIdx] = self.root + '/' + self.file_all[fIdx]
			self.mat2bag(self.fullpath[fIdx],vid_name,time_name,debug)
			print "Files left: " , (self.n_file-fIdx-1)
			print('')
		
		print('-------------------------DONE-------------------------')
	
	def batch_vid(self, rootdir, ext, debug):
	# For folder of video files (.mp4, .avi, .mov)
		self.get_files(rootdir,ext)
		self.fullpath = {}
		for fIdx in range(self.n_file):
			self.fullpath[fIdx] = self.root + '/' + self.file_all[fIdx]
			self.vid2bag(self.fullpath[fIdx],debug)
			print "Files left: ", (self.n_file - fIdx - 1)
			print('')
		
		print('-------------------------DONE-------------------------')
		
#if __name__ == '__main__':
	#main = BagTransform()
	#main.batch_mat('/home/jean-michel/BC/test','vidData','t',True)
	#main.batch_vid('/home/jean-michel/BC/test', '*.avi', True)