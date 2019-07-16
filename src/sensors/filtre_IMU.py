#!/usr/bin/env python2

import numpy as np
from numpy import cos,sin
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy import optimize

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import time
import os

from quaternion import Quaternion
from filter_lib import *

def get_raw_angles(raw_imu):
	ax,ay,az = raw_imu[0,0],raw_imu[1,0],raw_imu[2,0]
	gx,gy,gz = raw_imu[3,0],raw_imu[4,0],raw_imu[5,0]
	mx,my,mz = raw_imu[6,0],raw_imu[7,0],raw_imu[8,0]

	# ----  Calcul des angles ------------------------------------------------------ #

	A_pitch = np.arctan2(ay,az)
	A_roll = np.arctan2(-ax,np.sqrt(ay**2+az**2))
	Mx = mx*cos(A_pitch)+mz*sin(A_pitch)
	My = mx*sin(A_roll)*sin(A_pitch)+my*cos(A_roll)-mz*sin(A_roll)*cos(A_pitch)
	G_yaw = np.arctan2(-My,Mx)

	return G_yaw,A_pitch,A_roll

##############################################################################################
#      ROS
##############################################################################################

# http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
def sub_imu_nf(data):
	global vect_imu, vect_temps, get
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	vect_imu[0,0] = data.linear_acceleration.x # ax
	vect_imu[1,0] = data.linear_acceleration.y # ay
	vect_imu[2,0] = data.linear_acceleration.z # az
	vect_imu[3,0] = data.angular_velocity.x    # gx
	vect_imu[4,0] = data.angular_velocity.y    # gy
	vect_imu[5,0] = data.angular_velocity.z    # gz
	t = data.header.stamp.nsecs
	vect_temps = np.array([vect_temps[1],t,(t-vect_temps[1])/1000.])
	get = 1

def sub_mag_nf(data):
	global vect_imu
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	vect_imu[6,0] = data.magnetic_field.x         # mx
	vect_imu[7,0] = data.magnetic_field.y         # my
	vect_imu[8,0] = data.magnetic_field.z         # mz


##############################################################################################
#      Calibration
##############################################################################################

def resSphere(p,x,y,z):
	""" residuals from sphere fit """

	a,b,c,r = p                             # a,b,c are center x,y,c coords to be fit, r is the radius to be fit
	distance = np.sqrt( (x-a)**2 + (y-b)**2 + (z-c)**2 )
	err = distance - r                 # err is distance from input point to current fitted surface

	return err

def calibrate_imu(n = 200):# number of points to calibrate

	# --- Get data ---------------------------------------------------------- #
	mx,my,mz = np.zeros(n), np.zeros(n), np.zeros(n)
	counter = 0
	while counter < n: # while there is not enough values
		a = vect_imu.copy()
		rospy.sleep(0.01)
		if not(np.all(a==vect_imu)): #each time we have a new value
			mx[counter],my[counter],mz[counter] = a[6,0],a[7,0],a[8,0] # the value is added
			counter += 1
			rospy.loginfo(counter)

	params = [0.,0.,0.,0.]
	myResult = optimize.leastsq(resSphere, params, args=(mx,my,mz) )
	ox, oy, oz, r = myResult[0]
	rospy.loginfo(myResult[0])

	# ---- Save in text file ------------------------------------------------- #
	THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
	my_file = os.path.join(THIS_FOLDER, 'imu_previous_calibration.txt')
	f = open(my_file,'w')
	#f.write('offset_x:{}\noffset_y:{}\noffset_z:{}'.format(np.mean(mx), np.mean(my), np.mean(mz)))
	f.write('offset_x:{}\noffset_y:{}\noffset_z:{}'.format(-ox,-oy,-oz))
	f.close()
	offset = np.zeros((9,1))
	offset[6,0],offset[7,0],offset[8,0] = -ox,-oy,-oz
	return offset

def get_previous_imu():
	THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
	my_file = os.path.join(THIS_FOLDER, 'imu_previous_calibration.txt')
	f = open(my_file,'r')
	doc = f.readlines()
	f.close()
	offset = np.zeros((9,1))
	for i in range(len(doc)):
		offset[i+6] = float(doc[i].split(':')[1])
	#rospy.loginfo(offset)
	return offset

##############################################################################################
#      Filtre
##############################################################################################

# ----  EKF plus dur --------------------------------------------------------- #

def f(x,u):
	dt = vect_temps[2]	
	mat = np.array([[1,-dt*u,0],[dt*u,1,0],[0,0,1]])
	mat = np.matmul(mat,x)
	return mat

def F(x,u):
	dt = vect_temps[2]
	mat = np.array([[1,-dt*u,0],[dt*u,1,0],[0,0,1]])
	return mat

def h(x):
	x1,x2 = x[0,0],x[1,0]
	mat = np.array([[x1],[x2],[x1**2+x2**2]])
	return mat

def H(x):
	x1,x2 = x[0,0],x[1,0]
	mat = np.array([[1,0,0],[0,1,0],[2*x1,2*x2,0]])
	return mat

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

def display_mag(offset, scale=1):

	fig = plt.figure()
	ax = plt.axes(projection='3d')
	l = []

	t0 = time.time()
	while not rospy.is_shutdown():
		m = (vect_imu + offset)*scale
		mx,my,mz = m[6,0],m[7,0],m[8,0]
		l.append([mx,my,mz])
		clear(ax)
		ax.plot([0,mx],[0,my],[0,mz],'r')
		ax.plot([0,mx],[0,0],[0,0],'b')
		ax.plot([mx,mx],[0,my],[0,0],'b')
		ax.plot([mx,mx],[my,my],[0,mz],'b')
		ax.plot([0,10],[0,0],[0,0],'g')
		ax.plot([0,0],[0,5],[0,0],'g')
		ax.plot([0,0],[0,0],[0,5],'g')
		for i in l:
			ax.scatter(i[0],i[1],i[2])
		#rospy.loginfo(np.arctan2(my,mx)*180/np.pi)
		rospy.loginfo(np.sqrt(mx**2+my**2+mz**2))

##############################################################################################
#      Main
##############################################################################################

vect_imu = np.zeros((9,1)) # ax;ay;az;gx;gy;gz;mx;my;mz
vect_temps = np.array([0,0,0]) # t_prec, t_curr, dt
get = 0

if __name__ == '__main__':
	# kalman : https://pdfs.semanticscholar.org/7274/0405dcf42373fd4ed6fbd0a6f2d8d2208590.pdf

	# Three calibration modes : 
	# 'None' for no calibration, 'New' for new calibration, 'Previous' to take the previous one
	calibration = 'Previous'
	rospy.init_node('filtre_imu')
	rospy.Subscriber("ardu_send_imu", Imu, sub_imu_nf)
	rospy.Subscriber("ardu_send_mag", MagneticField, sub_mag_nf)
	if calibration == 'Previous':
		offset = get_previous_imu()
	elif calibration == 'New':
		offset = calibrate_imu()
	else:
		offset = np.zeros((9,1))
	rospy.loginfo("\nCalibration done")

	raw_imu = vect_imu + offset

	pub_send_euler_angles = rospy.Publisher('filter_send_euler_angles', Vector3, queue_size=10)
	euler_angles_msg = Vector3()

	P0 = 10*np.eye(3)
	Q = 0.028**2*np.eye(3)#0.028
	R = 0.01*np.eye(3)


	EKF_yaw   = Extended_kalman_filter(np.zeros((3,1)),P0,f,F,h,H,Q,R)
	EKF_pitch = Extended_kalman_filter(np.zeros((3,1)),P0,f,F,h,H,Q,R)
	EKF_roll  = Extended_kalman_filter(np.zeros((3,1)),P0,f,F,h,H,Q,R)

	q = Quaternion(1,0,0,0)

	alpha = 0.2
	low_pass_imu = raw_imu.copy()
	rospy.loginfo("\nStart main loop")
	while not rospy.is_shutdown():
		raw_imu = vect_imu + offset
		if get == 1: #each time we have a new value

			#t0 = time.time()
			get = 0
			low_pass_imu = alpha*raw_imu +(1-alpha)*low_pass_imu
			a = get_raw_angles(low_pass_imu)

			z = np.array([[np.cos(a[0])],[np.sin(a[0])],[1]])
			[x,P] = EKF_yaw.EKF_step(-low_pass_imu[5,0],z)
			s_yaw = np.arctan2(x[1,0],x[0,0])

			z = np.array([[np.cos(a[0])],[np.sin(a[0])],[1]])
			[x,P] = EKF_yaw.EKF_step(-raw_imu[5,0],z)
			yaw = np.arctan2(x[1,0],x[0,0])

			z = np.array([[np.cos(a[1])],[np.sin(a[1])],[1]])
			[x,P] = EKF_pitch.EKF_step(low_pass_imu[3,0],z)
			pitch = np.arctan2(x[1,0],x[0,0])-0.018

			z = np.array([[np.cos(a[2])],[np.sin(a[2])],[1]])
			[x,P] = EKF_roll.EKF_step(low_pass_imu[4,0],z)
			roll = np.arctan2(x[1,0],x[0,0])+0.008

			euler_angles_msg.x = yaw
			euler_angles_msg.y = pitch
			euler_angles_msg.z = roll
			pub_send_euler_angles.publish(euler_angles_msg)
			#rospy.loginfo(low_pass_imu[0,0]**2+low_pass_imu[1,0]**2+low_pass_imu[2,0]**2)
			#t1 = time.time()
			#rospy.loginfo("Yaw : {},Pitch : {}, Roll : {}".format(yaw*180/np.pi,pitch*180/np.pi,roll*180/np.pi))
			#rospy.loginfo(t1-t0)
			#plt.xlim((-1,1))
			#plt.ylim((-1,1))
			#plt.plot([0,cos(a[0])],[0, sin(a[0])],color = 'black')
			#plt.plot([0,cos(yaw)],[0, sin(yaw)],color = 'blue')
			#plt.pause(0.01)
			#plt.cla() 

			dt = vect_temps[2]
			wx,wy,wz = raw_imu[3,0],raw_imu[4,0],raw_imu[5,0]
			qdot = (Quaternion(0,raw_imu[3,0],raw_imu[4,0],raw_imu[5,0])*q)*0.5
			q = q + qdot*dt
			rospy.loginfo(q.to_euler123())





