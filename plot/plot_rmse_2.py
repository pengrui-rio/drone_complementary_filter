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
from matplotlib.ticker import MultipleLocator, FormatStrFormatter

from math import *
import csv
import math






def read_csv(path):
	csv_info = []

	csvFile = open(path, "r")
	reader = csv.reader(csvFile)


	for item in reader:
		csv_info.append(item)

	csvFile.close()

	result_data = []
	for i in range(len(csv_info)):
		if len(csv_info[i]) == 0:
			print "empty"
			continue

		p = []
		p.append(float(csv_info[i][0]))
		p.append(float(csv_info[i][1]))
		p.append(float(csv_info[i][2]))
		p.append(float(csv_info[i][3]))
		p.append(float(csv_info[i][4]))

		result_data.append(p)
		print p

	return result_data


def write_csv(PC_path, trajectoryInfo):
    csvFile = open(PC_path, "w")
    writer = csv.writer(csvFile)
    for i in range(len(trajectoryInfo)):
        writer.writerow(trajectoryInfo[i])
    csvFile.close()


if __name__ == '__main__':
	rospy.init_node('plt_display', anonymous=True)

	# file_name = "1-yaw.csv"
	# result_data = read_csv("/home/rio/Documents/arclab/18.validation/csv/" + file_name)

	# seq_list    = []
	# gt_data     = []
	# agstf_data  = []
	# ekf_data    = []
	# eskf_data   = []

	# result_inpaper = []
	# for i in range(len(result_data)):
	# 	if result_data[i][0] >= 30 and result_data[i][0] <= 90:
	# 		seq_list.append(float(result_data[i][0]))
	# 		gt_data.append(float(result_data[i][1]))
	# 		agstf_data.append(float(result_data[i][2]))
	# 		ekf_data.append(float(result_data[i][3]))
	# 		eskf_data.append(float(result_data[i][4]))

	# 		result_inpaper.append(result_data[i])

	# write_csv("/home/rio/Documents/arclab/18.validation/src/plot/csv_inpaper/" + file_name, result_inpaper)

	# plt.ion()
	# plt.figure(figsize = (6, 6), dpi = 80)
	# plt.figure(1)
	##### plt.title('plt_display')

	# plt.plot(seq_list, gt_data    , color='red'	   , linestyle = '-' , label='linear_acceleration_y') 
	# plt.plot(seq_list, agstf_data , color='blue' , linestyle = '-' , label='linear_acceleration_y') 
	# plt.plot(seq_list, ekf_data   , color='green'  , linestyle = '-' , label='linear_acceleration_y') 
	# plt.plot(seq_list, eskf_data  , color='black' , linestyle = '-' , label='linear_acceleration_y') 
	# plt.plot(seq_list, linear_acceleration_y_list, color='skyblue', linestyle = ':', label='linear_acceleration_y') 
	# plt.plot(seq_list, linear_acceleration_z_list, color='purple' , linestyle = ':', label='linear_acceleration_z') 


	# #########################################################################################################
	list_min = 30
	list_max = 60
	word_size = 35


	plt.ion()
	# plt.figure(figsize = (6, 6), dpi = 80)
	plt.figure(1)

	seq_list    = []
	gt_data     = []
	agstf_data  = []
	ekf_data    = []
	eskf_data   = []

	file_name = "2-yaw.csv"
	result_data = read_csv("/home/rio/Documents/arclab/18.validation/csv/" + file_name)
	for i in range(len(result_data)):
		if result_data[i][0] >= list_min and result_data[i][0] <= list_max:
			seq_list.append(float(result_data[i][0]))
			gt_data.append(float(result_data[i][1]))
			agstf_data.append(float(result_data[i][2]))
			ekf_data.append(float(result_data[i][3]))
			eskf_data.append(float(result_data[i][4]))

	ax1 = plt.subplot(313)
	plt.plot(seq_list, gt_data    , color='red'	   , linestyle = '-' , label=' ') 
	plt.plot(seq_list, agstf_data , color='blue'   , linestyle = '-' , label=' ') 
	plt.plot(seq_list, ekf_data   , color='green'  , linestyle = '-' , label=' ') 
	plt.plot(seq_list, eskf_data  , color='orange' , linestyle = '-' , label=' ') 

	plt.xlabel("Time(s)",fontsize=word_size,fontweight='bold')
	plt.ylabel("Yaw(deg)",fontsize=word_size,fontweight='bold')

	plt.xticks(fontsize=word_size)
	plt.yticks(fontsize=word_size)

	xmajorLocator = MultipleLocator(10) #将x主刻度标签设置为20的倍数
	ymajorLocator = MultipleLocator(6) #将y轴主刻度标签设置为0.5的倍数
	ax1.xaxis.set_major_locator(xmajorLocator)
	ax1.yaxis.set_major_locator(ymajorLocator)

	# ax1.yaxis.set_label_coords(0, 0.5)

	############################################################################
	seq_list    = []
	gt_data     = []
	agstf_data  = []
	ekf_data    = []
	eskf_data   = []

	file_name = "2-pitch.csv"
	result_data = read_csv("/home/rio/Documents/arclab/18.validation/csv/" + file_name)
	for i in range(len(result_data)):
		if result_data[i][0] >= list_min and result_data[i][0] <= list_max:
			seq_list.append(float(result_data[i][0]))
			gt_data.append(float(result_data[i][1]))
			agstf_data.append(float(result_data[i][2]))
			ekf_data.append(float(result_data[i][3]))
			eskf_data.append(float(result_data[i][4]))

	ax2 = plt.subplot(312)
	plt.plot(seq_list, gt_data    , color='red'	   , linestyle = '-' , label=' ') 
	plt.plot(seq_list, agstf_data , color='blue'   , linestyle = '-' , label=' ') 
	plt.plot(seq_list, ekf_data   , color='green'  , linestyle = '-' , label=' ') 
	plt.plot(seq_list, eskf_data  , color='orange' , linestyle = '-' , label=' ') 

	plt.ylabel("Pitch(deg)",fontsize=word_size,fontweight='bold')
	plt.xticks(fontsize=word_size)
	plt.yticks(fontsize=word_size)

	xmajorLocator = MultipleLocator(10) #将x主刻度标签设置为20的倍数
	ymajorLocator = MultipleLocator(20) #将y轴主刻度标签设置为0.5的倍数
	ax2.xaxis.set_major_locator(xmajorLocator)
	ax2.yaxis.set_major_locator(ymajorLocator)
 
	ax2.yaxis.set_label_coords(-0.057, 0.5)
	############################################################################
	seq_list    = []
	gt_data     = []
	agstf_data  = []
	ekf_data    = []
	eskf_data   = []

	file_name = "2-roll.csv"
	result_data = read_csv("/home/rio/Documents/arclab/18.validation/csv/" + file_name)
	for i in range(len(result_data)):
		if result_data[i][0] >= list_min and result_data[i][0] <= list_max:
			seq_list.append(float(result_data[i][0]))
			gt_data.append(float(result_data[i][1]))
			agstf_data.append(float(result_data[i][2]))
			ekf_data.append(float(result_data[i][3]))
			eskf_data.append(float(result_data[i][4]))

	ax3 = plt.subplot(311)
	plt.plot(seq_list, gt_data    , color='red'	  , linestyle = '-' , label='VICON reference') 
	plt.plot(seq_list, agstf_data , color='blue'  , linestyle = '-' , label='Proposed filter') 
	plt.plot(seq_list, ekf_data   , color='green' , linestyle = '-' , label='EKF') 
	plt.plot(seq_list, eskf_data  , color='orange' , linestyle = '-' , label='ESKF') 

	plt.ylabel("Roll(deg)",fontsize=word_size,fontweight='bold')
	plt.xticks(fontsize=word_size)
	plt.yticks(fontsize=word_size)

	xmajorLocator = MultipleLocator(10) #将x主刻度标签设置为20的倍数
	# xmajorFormatter = FormatStrFormatter('%5.1f') #设置x轴标签文本的格式
	ymajorLocator = MultipleLocator(25) #将y轴主刻度标签设置为0.5的倍数
	# ymajorFormatter = FormatStrFormatter('%5.1f') #设置y轴标签文本的格式
	#设置主刻度标签的位置,标签文本的格式
	ax3.xaxis.set_major_locator(xmajorLocator)
	# ax3.xaxis.set_major_formatter(xmajorFormatter)
	ax3.yaxis.set_major_locator(ymajorLocator)
	# ax3.yaxis.set_major_formatter(ymajorFormatter)


	# plt.legend(loc='upper center', fontsize=14, frameon=True, fancybox=True, framealpha=0.2, borderpad=0.3,
    #        ncol=ncol, markerfirst=True, markerscale=1, numpoints=1, handlelength=3.5)

	ncol = int(math.ceil(4))
	# ax3.legend(ncol=ncol, loc='upper center', fontsize=word_size, bbox_to_anchor=(0.5,1.38))

	leg = ax3.legend(ncol=ncol, loc='upper center', fontsize=word_size, bbox_to_anchor=(0.5,1.38))
	leg_lines = leg.get_lines()
	plt.setp(leg_lines, linewidth=5)


	# plt.tight_layout()#调整整体空白
	plt.subplots_adjust(wspace = 0.2,  hspace = 0.2, 
					    left   = 0.08, right  = 0.98, 
						bottom = 0.09, top    = 0.92)#调整图间距

	plt.show()
	plt.pause(100000)


