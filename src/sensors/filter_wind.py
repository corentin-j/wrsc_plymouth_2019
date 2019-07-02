#!/usr/bin/env python2

import numpy as np
from numpy import cos,sin
import matplotlib.pyplot as plt
import time

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

from filter_lib import *

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

##############################################################################################
#      ROS
##############################################################################################

def sub_wind_direction(data): # Float32
    global get, vect_temps, vect_wind_direction
    wind_direction = data.data
    #rospy.loginfo("wind_direction : %s", wind_direction)
    t = time.time()
    vect_temps = np.array([vect_temps[1],t,(t-vect_temps[1])])
    vect_wind_direction = np.array([vect_wind_direction[1],wind_direction,sawtooth(wind_direction-vect_wind_direction[1])])
    get = 1

def sub_euler_angles(data): # Vector3
    global theta
    theta = -data.x
    #rospy.loginfo("theta : %s",theta*180/np.pi)

##############################################################################################
#      Filtre
##############################################################################################

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

##############################################################################################
#      Main
##############################################################################################

vect_wind_direction = np.array([0,0,0])
get = 0
vect_temps = np.array([0,0,0]) # t_prec, t_curr, dt
theta = 0

if __name__ == '__main__':
	# kalman : https://pdfs.semanticscholar.org/7274/0405dcf42373fd4ed6fbd0a6f2d8d2208590.pdf
	rospy.init_node('filtre_wind_direction')

	rospy.Subscriber("filter_send_euler_angles", Vector3, sub_euler_angles)
	rospy.Subscriber("ardu_send_wind", Float32, sub_wind_direction)
	pub_send_wind_direction = rospy.Publisher('filter_send_wind_direction', Float32, queue_size=10)
	wind_direction_msg = Float32()

	P0 = 10*np.eye(3)
	Q = 0.028**2*np.eye(3)#0.028
	R = 0.01*np.eye(3)
	EKF_yaw   = Extended_kalman_filter(np.zeros((3,1)),P0,f,F,h,H,Q,R)
	
	l_temps,l_wind_raw,l_wind_EKF = [],[],[]

	while not rospy.is_shutdown():

		if get == 1: #each time we have a new value
			get = 0
			# ----  Extended Kalman Filter ------------------------------------------------- #

			z = np.array([[np.cos(vect_wind_direction[1])],[np.sin(vect_wind_direction[1])],[1]])
			[x,P] = EKF_yaw.EKF_step(vect_wind_direction[2],z)
			wind_direction = np.arctan2(x[1,0],x[0,0])

			wind_direction_msg.data = theta - wind_direction
			rospy.loginfo("vent absolu : {}".format(theta - wind_direction))
			pub_send_wind_direction.publish(wind_direction_msg)
			#rospy.loginfo("Yaw : {}".format(yaw*180/np.pi))

			l_temps.append(vect_temps[1])
			l_wind_raw.append(vect_wind_direction[1]*180/np.pi)
			l_wind_EKF.append(wind_direction*180/np.pi)

			#plt.xlim((-1,1))
			#plt.ylim((-1,1))
			#plt.plot([0,cos(wind_direction)],[0, sin(wind_direction)])
			#plt.plot([0,cos(vect_wind_direction[1])],[0, sin(vect_wind_direction[1])])
			#plt.pause(0.01)
			#plt.cla()



