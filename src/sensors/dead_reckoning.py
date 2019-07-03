#!/usr/bin/env python2

import numpy as np
from numpy import cos,sin
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import time

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import os

from filter_lib import *

def rotation_matrix(pitch, roll):

	R_pitch  = np.array([[cos(pitch),0,-sin(pitch)],[0,1,0],[sin(pitch),0,cos(pitch)]])
	R_roll   = np.array([[1,0,0],[0,cos(roll),sin(roll)],[0,-sin(roll),cos(roll)]])
	R_repere = np.array([[0,-1,0],[-1,0,0],[0,0,1]])

	return np.matmul(R_repere,np.matmul(R_pitch,R_roll))

##############################################################################################
#      ROS
##############################################################################################

def sub_imu_nf(data):
	global vect_imu, vect_temps, get
	g = 9.806
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	vect_imu[0,0] = data.linear_acceleration.x/g # ax
	vect_imu[1,0] = data.linear_acceleration.y/g # ay
	vect_imu[2,0] = data.linear_acceleration.z/g # az
	vect_imu[3,0] = data.angular_velocity.x      # gx
	vect_imu[4,0] = data.angular_velocity.y      # gy
	vect_imu[5,0] = data.angular_velocity.z      # gz
	t = data.header.stamp.nsecs
	vect_temps = np.array([vect_temps[1],t,(t-vect_temps[1])/1000.])
	get = 1

def sub_mag_nf(data):
	global vect_imu
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	vect_imu[6,0] = data.magnetic_field.x         # mx
	vect_imu[7,0] = data.magnetic_field.y         # my
	vect_imu[8,0] = data.magnetic_field.z         # mz

def sub_euler_angles(data): # Vector3
    global pitch, roll
    pitch = data.y
    roll = data.z
    #rospy.loginfo("theta : %s",theta*180/np.pi)

##############################################################################################
#      Display
##############################################################################################

def clear(ax):
    plt.pause(0.001)
    plt.cla()
    a = 15
    ax.set_xlim(-a,a)
    ax.set_ylim(-a,a)
    ax.set_zlim(-a,a)

def get_previous_imu():
	THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
	my_file = os.path.join(THIS_FOLDER, 'imu_previous_calibration.txt')
	f = open(my_file,'r')
	doc = f.readlines()
	f.close()
	offset = np.zeros((9,1))
	for i in range(len(doc)):
		offset[i+6] = float(doc[i].split(':')[1])
	rospy.loginfo(offset)
	return offset

def find_rot_matrix(n=20):
	global get
	offset = get_previous_imu()

	counter = 0
	s_R = np.zeros((3,3))
	while counter < n and not rospy.is_shutdown():
		if get == 1:
			get = 0
			m = vect_imu + offset
			ax,ay,az = m[0,0],m[1,0],m[2,0]
			mx,my,mz = m[6,0],m[7,0],m[8,0]

			nax,nay,naz = ay*200,ax*200,-az*200
			M,A = np.array([mx,my,mz]), np.array([nax,nay,naz])
			C = np.cross(M,A)
			D = np.dot(M,A)
			NP0 = np.linalg.norm(M)
			Z = np.array([[0,-C[2],C[1]],[C[2],0,-C[0]],[-C[1],C[0],0]])
			R = (np.eye(3) + Z + np.matmul(Z,Z) * (1-D)/(np.linalg.norm(C)**2)) / NP0**2
			#res = np.matmul(R,M)
			s_R += R
			#rospy.loginfo(R)
			counter += 1
	return s_R/n

def display_mag(scale=1):

	fig = plt.figure()
	axe = plt.axes(projection='3d')
	l = []

	offset = get_previous_imu()
	t0 = time.time()
	while not rospy.is_shutdown():
		m = (vect_imu + offset)*scale
		ax,ay,az = m[0,0],m[1,0],m[2,0]
		mx,my,mz = m[6,0],m[7,0],m[8,0]
		M = np.array([mx,my,mz])

		new_a = np.matmul(R,M)

		l.append([mx,my,mz])
		clear(axe)
		axe.plot([0,mx],[0,my],[0,mz],'r')
		axe.plot([0,mx],[0,0],[0,0],'b')
		axe.plot([mx,mx],[0,my],[0,0],'b')
		axe.plot([mx,mx],[my,my],[0,mz],'b')
		axe.plot([0,10],[0,0],[0,0],'g')
		axe.plot([0,0],[0,5],[0,0],'g')
		axe.plot([0,0],[0,0],[0,5],'g')
		axe.plot([0,ay*200],[0,ax*200],[0,-az*200],'c')
		axe.plot([0,new_a[0]],[0,new_a[1]],[0,new_a[2]])

		#for i in l:
		#	ax.scatter(i[0],i[1],i[2])
		#rospy.loginfo(np.arctan2(my,mx)*180/np.pi)
		

		rospy.loginfo([R])

##############################################################################################
#      Main
##############################################################################################

vect_imu = np.zeros((9,1)) # ax;ay;az;gx;gy;gz;mx;my;mz
get = 0
vect_temps = np.array([0,0,0]) # t_prec, t_curr, dt
pitch, roll = 0,0

if __name__ == '__main__':
	# kalman : https://pdfs.semanticscholar.org/7274/0405dcf42373fd4ed6fbd0a6f2d8d2208590.pdf
	rospy.init_node('dead_reckoning')
	rospy.Subscriber("ardu_send_imu", Imu, sub_imu_nf)
	rospy.Subscriber("ardu_send_mag", MagneticField, sub_mag_nf)

	dt = vect_temps[2]
	x = np.array([[0],[0]])
	P = np.zeros((2,2))
	F = np.array([[1,dt],[0,1]])
	B = np.array([[dt**2/2],[dt]])
	H = np.zeros((2,1))
	Q = 0.028**2*np.eye(2)#0.028
	R = 0.01*np.eye(2)

	kf_x = Kalman_filter(x,P,F,B,H,Q,R)

	R = find_rot_matrix()

	rospy.loginfo("Mean R : {}".format(R))

	while not rospy.is_shutdown():

		if get == 1: #each time we have a new value
			get = 0
			# ----  Extended Kalman Filter ------------------------------------------------- #
			dt = vect_temps[2]
			F = np.array([[1,dt],[0,1]])
			B = np.array([[dt**2/2],[dt]])
			kf_x.kalman_set_F(F)
			kf_x.kalman_set_B(B)

			ax,ay,az = vect_imu[0,0],vect_imu[1,0],vect_imu[2,0]
			R_mat = rotation_matrix(pitch,roll)
			acc_NED = np.dot(np.linalg.inv(R_mat),np.array([[ax],[ay],[az]]))

			[x,P] = kf_x.kalman_predict(acc_NED[0,0])

			display_mag()
			
			
			#rospy.loginfo(x)


			#plt.xlim((-1,1))
			#plt.ylim((-1,1))
			#plt.plot([0,cos(wind_direction)],[0, sin(wind_direction)])
			#plt.plot([0,cos(vect_wind_direction[1])],[0, sin(vect_wind_direction[1])])
			#plt.pause(0.01)
			#plt.cla()



