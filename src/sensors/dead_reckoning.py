#!/usr/bin/env python2

import numpy as np
from numpy import cos,sin
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import time

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
import os

from filter_lib import *

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

def rot_yaw(yaw):
	R_yaw = np.array([[cos(yaw),-sin(yaw),0],[sin(yaw),cos(yaw),0],[0,0,1]])
	return R_yaw

def rotation_matrix(pitch, roll, yaw=0):

	R_pitch  = np.array([[cos(pitch),0,-sin(pitch)],[0,1,0],[sin(pitch),0,cos(pitch)]])
	R_roll   = np.array([[1,0,0],[0,cos(roll),sin(roll)],[0,-sin(roll),cos(roll)]])
	R_yaw    = np.array([[cos(yaw),-sin(yaw),0],[sin(yaw),cos(yaw),0],[0,0,1]])
	#R_repere = np.array([[0,-1,0],[-1,0,0],[0,0,1]])

	return np.matmul(np.matmul(R_pitch,R_roll),R_yaw)

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

##############################################################################################
#      ROS
##############################################################################################

def sub_imu_nf(data):
	global vect_imu, vect_temps, get
	g = 9.806
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	vect_imu[0,0] = data.linear_acceleration.x # ax
	vect_imu[1,0] = data.linear_acceleration.y # ay
	vect_imu[2,0] = data.linear_acceleration.z # az
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
    global yaw, pitch, roll
    yaw = data.x
    pitch = data.y
    roll = data.z
    #rospy.loginfo(np.array([yaw,pitch,roll])*180/np.pi)

##############################################################################################
#      Display
##############################################################################################

def clear(ax):
    plt.pause(0.001)
    plt.cla()
    a = 10
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
	#rospy.loginfo(offset)
	return offset

def find_rotation_matrix(d,f):
	C = np.cross(d,f)
	D = np.dot(d,f)
	NP0 = np.linalg.norm(d)
	Z = np.array([[0,-C[2],C[1]],[C[2],0,-C[0]],[-C[1],C[0],0]])
	R = (np.eye(3) + Z + np.matmul(Z,Z) * (1-D)/(np.linalg.norm(C)**2)) / NP0**2
	return R

	

def init_rot_matrix(n=20):
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
			R = find_rotation_matrix(M,A)
			#res = np.matmul(R,M)
			s_R += R
			#rospy.loginfo(R)
			counter += 1
	return s_R/n

def offsets_gyro(n = 20):
	global get
	counter = 0
	lgx,lgy,lgz = [],[],[]
	while not rospy.is_shutdown() and counter < n:
		if get == 1: #each time we have a new value
			get = 0
			lgx.append(vect_imu[3,0])
			lgy.append(vect_imu[4,0])
			lgz.append(vect_imu[5,0])
			rospy.loginfo([lgx[-1],lgy[-1],lgz[-1]])
			counter += 1
	rospy.loginfo(("wala",np.mean(lgx), np.mean(lgy),np.mean(lgz)))
	o = np.zeros((9,1))
	o[3,0],o[4,0],o[5,0] = -np.mean(lgx), -np.mean(lgy), -np.mean(lgz)
	return o




def display_mag():

	fig = plt.figure()
	axe = plt.axes(projection='3d')
	l = []

	o_g = offsets_gyro()
	offset = get_previous_imu()
	t0 = time.time()
	m = vect_imu + offset + o_g
	ax,ay,az = m[0,0],m[1,0],m[2,0]
	mx,my,mz = m[6,0],m[7,0],m[8,0]
	p_ax,p_ay,p_az = ax,ay,az
	tst = np.array([ax,ay,az]).reshape((3,1))*200
	th = 0
	x=0
	v = 0

	while not rospy.is_shutdown():
		m = vect_imu + offset + o_g
		ax,ay,az = m[0,0],m[1,0],m[2,0]
		gx,gy,gz = m[3,0],m[4,0],m[5,0]
		mx,my,mz = m[6,0],m[7,0],m[8,0]
		dt = vect_temps[2]

		R = rotation_matrix(roll,pitch,yaw) # dt*gy, dt*gx, -dt*gz
		tst = np.matmul(R,np.array([[0],[0],[10]]))
		nv = np.matmul(np.linalg.inv(R),np.array([[ax],[ay],[az]]))
		nv = (nv-np.array([[0],[0],[9.81]]))

		clear(axe)
		axe.plot([0,mx],[0,my],[0,mz],'r')
		axe.plot([0,ax],[0,0],[0,0],'b')
		axe.plot([ax,ax],[0,ay],[0,0],'b')
		axe.plot([ax,ax],[ay,ay],[0,az],'b')
		axe.plot([0,10],[0,0],[0,0],'g')
		axe.plot([0,0],[0,5],[0,0],'g')
		axe.plot([0,0],[0,0],[0,5],'g')
		axe.plot([0,ax*200],[0,ay*200],[0,az*200],'c')
		a_rest = np.array([ax,ay,az])
		axe.plot([0,tst[0,0]],[0,tst[1,0]],[0,tst[2,0]],color="purple")
		axe.plot([0,nv[0,0]*50],[0,nv[1,0]*50],[0,nv[2,0]*50], color="orange")
		#time.sleep(1)
		rospy.loginfo([np.arctan2(my,mx)*180/np.pi])
		v += dt*nv[0,0]
		x += v*dt
		


		p_ax,p_ay,p_az = ax,ay,az


		



##############################################################################################
#      Main
##############################################################################################

vect_imu = np.zeros((9,1)) # ax;ay;az;gx;gy;gz;mx;my;mz
get = 0
vect_temps = np.array([0,0,0]) # t_prec, t_curr, dt
yaw,pitch,roll = 0,0,0

if __name__ == '__main__':
	# kalman : https://pdfs.semanticscholar.org/7274/0405dcf42373fd4ed6fbd0a6f2d8d2208590.pdf
	rospy.init_node('dead_reckoning')
	rospy.Subscriber("ardu_send_imu", Imu, sub_imu_nf)
	rospy.Subscriber("ardu_send_mag", MagneticField, sub_mag_nf)
	rospy.Subscriber("ardu_send_euler", Vector3, sub_euler_angles)

	dt = vect_temps[2]
	x = np.array([[0],[0]])
	P = np.zeros((2,2))
	F = np.array([[1,dt],[0,1]])
	B = np.array([[dt**2/2],[dt]])
	H = np.zeros((2,1))
	Q = 0.028**2*np.eye(2)#0.028
	R = 0.01*np.eye(2)

	kf_x = Kalman_filter(x,P,F,B,H,Q,R)

	#R0 = init_rot_matrix()

	
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
			

			#[x,P] = kf_x.kalman_predict(acc_NED[0,0])

			display_mag()

			
			R_mat = rotation_matrix(pitch,roll,yaw)
			v = np.matmul(np.linalg.inv(R_mat),np.array([[ax],[ay],[az]]))
			


			#plt.xlim((-1,1))
			#plt.ylim((-1,1))
			#plt.plot([0,cos(wind_direction)],[0, sin(wind_direction)])
			#plt.plot([0,cos(vect_wind_direction[1])],[0, sin(vect_wind_direction[1])])
			#plt.pause(0.01)
			#plt.cla()