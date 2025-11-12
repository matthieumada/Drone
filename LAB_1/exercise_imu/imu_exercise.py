#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# IMU exercise
# Copyright (c) 2015-2024 Kjeld Jensen kjen@mmmi.sdu.dk kj@kjen.dk

##### Insert initialize code below ###################

## Uncomment the file to read ##
fileName = 'imu_razor_data_static.txt'
fileName = 'imu_razor_data_pitch_55deg.txt'
#fileName = 'imu_razor_data_roll_65deg.txt'
#fileName = 'imu_razor_data_yaw_90deg.txt'

## IMU type
#imuType = 'vectornav_vn100'
imuType = 'sparkfun_razor'

## Variables for plotting ##
showPlot = True
plotData1 = []
plotData2 = []

## Initialize your variables here ##
myValue_p = 0.0
myValue_r = 0.0
sn_r = 0.0
sn_p = 0.0
angle_z = 0.0

######################################################

# import libraries
from math import pi, sqrt, atan2
from numpy import mean 
import matplotlib
matplotlib.use("TkAgg")   # ou "Qt5Agg" si tu as PyQt5
import matplotlib.pyplot as plt


# open the imu data file
f = open (fileName, "r")

# initialize variables
count = 0

# looping through file

for line in f:
	count += 1

	# split the line into CSV formatted data
	line = line.replace ('*',',') # make the checkum another csv value
	csv = line.split(',')

	# keep track of the timestamps 
	ts_recv = float(csv[0])
	if count == 1: 
		ts_now = ts_recv # only the first time
	ts_prev = ts_now
	ts_now = ts_recv

	if imuType == 'sparkfun_razor':
		# import data from a SparkFun Razor IMU (SDU firmware)
		acc_x = int(csv[2]) / 1000.0 * 4 * 9.82;
		acc_y = int(csv[3]) / 1000.0 * 4 * 9.82;
		acc_z = int(csv[4]) / 1000.0 * 4 * 9.82;
		gyro_x = int(csv[5]) * 1/14.375 * pi/180.0;
		gyro_y = int(csv[6]) * 1/14.375 * pi/180.0;
		gyro_z = int(csv[7]) * 1/14.375 * pi/180.0;

	elif imuType == 'vectornav_vn100':
		# import data from a VectorNav VN-100 configured to output $VNQMR
		acc_x = float(csv[9])
		acc_y = float(csv[10])
		acc_z = float(csv[11])
		gyro_x = float(csv[12])
		gyro_y = float(csv[13])
		gyro_z = float(csv[14])
	 		
	##### Insert loop code below #########################

	# Variables available
	# ----------------------------------------------------
	# count		Current number of updates		
	# ts_prev	Time stamp at the previous update
	# ts_now	Time stamp at this update
	# acc_x		Acceleration measured along the x axis
	# acc_y		Acceleration measured along the y axis
	# acc_z		Acceleration measured along the z axis
	# gyro_x	Angular velocity measured about the x axis
	# gyro_y	Angular velocity measured about the y axis
	# gyro_z	Angular velocity measured about the z axis

	## Insert your code here ##
	roll = atan2(-acc_x , acc_z)
	pitch = atan2(acc_y,sqrt(acc_x*acc_x + acc_z*acc_z))
		
	### -- Filtering  --
	alpha = 0.1
	# pitch 
	#sn_p = pitch * alpha + (1-alpha)*myValue_p

	# roll 
	#sn_r = roll *alpha + (1-alpha)*myValue_r

	### -- Integration -- 
	dt = ts_now - ts_prev
	# pitch 
	angle_z = angle_z + gyro_z * dt 

	# in order to show a plot use this function to append your value to a list:
	plotData1.append(angle_z*180.0/pi)
	plotData2.append(pitch*180.0/pi)
	#myValue_p = sn_p 
	#myValue_r = sn_r

	######################################################

# closing the file	
f.close()

def variance(data,m):
	z = 0.0
	for i in data : 
		z = z + (i - m)**2
	return sqrt(z/len(data))


# show the plot
if showPlot == True:
	print('Mean gyro z (bias): ',mean(plotData2),' deg/s')
	print('Variance gyro : ',variance(plotData2,mean(plotData2)),' deg/s')
	plt.figure()
	plt.plot(plotData1, label = 'Pitch')
	plt.xlabel('Sample')
	plt.ylabel('Angle [°]')
	plt.title('Angle from gyroscope from: '+fileName)
	plt.legend()
	plt.grid('on')
	#plt.savefig('imu_exercise_plot.png') 

	plt.figure()
	plt.plot(plotData2, label = 'Velocity')
	plt.xlabel('Sample')
	plt.ylabel('Angle [°]')
	plt.title('Velocity from gyroscope from: '+fileName)
	plt.legend()
	plt.grid('on')

	plt.show()




