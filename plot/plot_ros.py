#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np
import time
import sys
import copy
import rospy
import geometry_msgs.msg
from math import pi
from geometry_msgs.msg import PoseStamped,Twist,TransformStamped,QuaternionStamped
from sensor_msgs.msg import Imu
import matplotlib as mpl
import matplotlib.pyplot as plt
from math import *




# seq_list_gt    = []
# gt_data        = []

# seq_list_agstf = []
# agstf_data     = []

# seq_list_ekf   = []
# ekf_data       = []

# seq_list_eskf  = []
# eskf_data      = []

seq_list    = []
gt_data        = []
agstf_data     = []
ekf_data       = []
eskf_data      = []

def callback_comparison(pose):
	global seq_list
	global gt_data
	global agstf_data
	global ekf_data
	global eskf_data
	seq_list.append(pose.header.seq)

	gt_data.append(pose.pose.orientation.w)
	agstf_data.append(pose.orientation.x)
	ekf_data.append(pose.orientation.y)
	eskf_data.append(pose.orientation.z)

# def callback_gt_data(pose):
# 	global seq_list_gt
# 	global gt_data
# 	seq_list_gt.append(pose.header.seq)

# 	gt_data.append(pose.pose.orientation.x)
# 	# gt_data.append(pose.orientation.y)
# 	# gt_data.append(pose.orientation.z)

# def callback_agstf_data(pose):
# 	global seq_list_agstf
# 	global agstf_data
# 	seq_list_agstf.append(pose.header.seq)

# 	agstf_data.append(pose.pose.orientation.x)
# 	# agstf_data.append(pose.orientation.y)
# 	# agstf_data.append(pose.orientation.z)

# def callback_ekf_data(pose):
# 	global seq_list_ekf
# 	global ekf_data
# 	seq_list_ekf.append(pose.header.seq)

# 	ekf_data.append(pose.pose.orientation.x)
# 	# ekf_data.append(pose.orientation.y)
# 	# ekf_data.append(pose.orientation.z)

# def callback_eskf_data(pose):
# 	global seq_list_eskf
# 	global eskf_data
# 	seq_list_eskf.append(pose.header.seq)

# 	eskf_data.append(pose.pose.orientation.x)
# 	# eskf_data.append(pose.orientation.y)
# 	# eskf_data.append(pose.orientation.z)


if __name__ == '__main__':
	rospy.init_node('plt_display', anonymous=True)

	rospy.Subscriber("comparison"   , PoseStamped, callback_comparison)
	# rospy.Subscriber("GT_pose_publish"   , PoseStamped, callback_gt_data)
	# rospy.Subscriber("AGSTF_pose_publish", PoseStamped, callback_agstf_data)
	# rospy.Subscriber("EKF_pose_publish"  , PoseStamped, callback_ekf_data)
	# rospy.Subscriber("ESKF_pose_publish" , PoseStamped, callback_eskf_data)

	time_start = time.time()
	time_count = []

	plt.ion()
	plt.figure(1)
	plt.title('plt_display')

	legend_show_flag = 1
	while not rospy.is_shutdown():

		time_end = time.time()
		print('time cost', time_end-time_start ,'s')
		time_count.append(time_end-time_start)

		plt.plot(seq_list, gt_data    , color='red'	   , linestyle = '-', label='gt_data') 
		plt.plot(seq_list, agstf_data , color='blue' , linestyle = '-', label='agstf_data') 
		plt.plot(seq_list, ekf_data   , color='green'  , linestyle = '-', label='ekf_data') 
		plt.plot(seq_list, eskf_data  , color='black' , linestyle = '-', label='eskf_data') 
		# plt.plot(seq_list, linear_acceleration_y_list, color='skyblue', linestyle = ':', label='linear_acceleration_y') 
		# plt.plot(seq_list, linear_acceleration_z_list, color='purple' , linestyle = ':', label='linear_acceleration_z') 


		if legend_show_flag == 1:
			legend_show_flag = 0
			plt.legend()
			plt.show()

		plt.pause(0.00001)

