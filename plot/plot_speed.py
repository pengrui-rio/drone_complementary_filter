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
from matplotlib.patches import Polygon

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



	list_min = 0
	list_max = 500
	word_size = 35


	plt.ion()
	plt.figure(1)

	seq_list    = []
	agstf_data  = []
	ekf_data    = []
	eskf_data   = []

	file_name = "speed.csv"
	result_data = read_csv("/home/rio/Documents/arclab/18.validation/csv/" + file_name)
	for i in range(len(result_data)):
		if result_data[i][0] >= list_min and result_data[i][0] <= list_max:
			seq_list.append(float(result_data[i][0]))
			agstf_data.append(float(result_data[i][1])*1000+0.05)
			ekf_data.append(float(result_data[i][2])*1000)
			eskf_data.append(float(result_data[i][3])*1000)

	ax1 = plt.subplot(111)
	plt.plot(seq_list, agstf_data , color='blue'  , linestyle = '-' , label='Proposed filter') 
	plt.plot(seq_list, ekf_data   , color='green' , linestyle = '-' , label='EKF') 
	plt.plot(seq_list, eskf_data  , color='orange' , linestyle = '-' , label='ESKF') 

	plt.xlabel("Epoch($i^{th}$)", fontsize=word_size, fontweight='bold')
	plt.ylabel("Runtime(ms)",     fontsize=word_size, fontweight='bold')

	plt.xticks(fontsize=word_size)
	plt.yticks(fontsize=word_size)

	#设置坐标轴范围
	# plt.xlim((-5, 5))
	plt.ylim((-0.2, 2))

	xmajorLocator = MultipleLocator(250) #将x主刻度标签设置为20的倍数
	ax1.xaxis.set_major_locator(xmajorLocator)
	ymajorLocator = MultipleLocator(0.6) #将x主刻度标签设置为20的倍数
	ax1.yaxis.set_major_locator(ymajorLocator)


	ncol = int(math.ceil(3))
	leg = ax1.legend(ncol=ncol, loc='upper center', fontsize=word_size, bbox_to_anchor=(0.5, 1.11))
	leg_lines = leg.get_lines()
	plt.setp(leg_lines, linewidth=5)


	# plt.tight_layout()#调整整体空白
	plt.subplots_adjust(wspace = 0.2,  hspace = 0.2, 
					    left   = 0.07, right  = 0.97, 
						bottom = 0.095, top    = 0.92)#调整图间距


	plt.show()
	plt.pause(100000)


