
from BagTransform import BagTransform

RUN = BagTransform()

root 		= '/media/jean-michel/My Book/EXPERIMENTS/Experiment_SOS_v2'
vid_name 	= 'vidData'
time_name 	= 't_v'
debug 		= False

RUN.batch_mat(root, vid_name, time_name, debug)