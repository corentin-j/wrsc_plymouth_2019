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

def sub_euler_angles(data): # Vector3
    global yaw, pitch, roll
    yaw = data.x
    pitch = data.y
    roll = data.z
    #rospy.loginfo(np.array([yaw,pitch,roll])*180/np.pi)




##############################################################################################
#      Main
##############################################################################################

vect_imu = np.zeros((9,1)) # ax;ay;az;gx;gy;gz;mx;my;mz
get = 0
vect_temps = np.array([0,0,0]) # t_prec, t_curr, dt
yaw,pitch,roll = 0,0,0

if __name__ == '__main__':
	# kalman : https://pdfs.semanticscholar.org/7274/0405dcf42373fd4ed6fbd0a6f2d8d2208590.pdf
	rospy.init_node('cmps')
	rospy.Subscriber("ardu_send_imu", Imu, sub_imu_nf)
	rospy.Subscriber("ardu_send_euler", Vector3, sub_euler_angles)

	vx = 0
	time.sleep(2)
	
	while not rospy.is_shutdown():

		if get == 1: #each time we have a new value
			get = 0


			#display_mag()
			ax,ay,az = vect_imu[0,0],vect_imu[1,0],vect_imu[2,0]
			dt = vect_temps[2]

			
			R = rotation_matrix(roll,pitch,yaw) # dt*gy, dt*gx, -dt*gz
			tst = np.matmul(R,np.array([[0],[0],[10]]))
			nv = np.matmul(np.linalg.inv(R),np.array([[ax],[ay],[az]]))
			nv = (nv-np.array([[0],[0],[9.81]]))
			#if np.linalg.norm(nv)<10:
			vx += nv.flatten()[0]*dt
			rospy.loginfo(nv.flatten())


			


			#plt.xlim((-1,1))
			#plt.ylim((-1,1))
			#plt.plot([0,cos(wind_direction)],[0, sin(wind_direction)])
			#plt.plot([0,cos(vect_wind_direction[1])],[0, sin(vect_wind_direction[1])])
			#plt.pause(0.01)
			#plt.cla()