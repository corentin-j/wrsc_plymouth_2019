#!/usr/bin/env python2

import numpy as np
from numpy import cos,sin
import matplotlib.pyplot as plt
import time

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from gps_common.msg import GPSFix

import sys
import rospkg
rospack = rospkg.RosPack()
pkg = rospack.get_path('wrsc_plymouth_jegat')
sys.path.append(pkg+'/src/my_libs')
from filter_lib import *

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

##############################################################################################
#      ROS
##############################################################################################

def sub_wind_direction(data): # Float32
	#rospy.loginfo("[{}] I see wind direction".format(node_name))
	rospy.sleep(0.01)
	global get, vect_temps, vect_wind_direction
	wind_direction = data.data
	#rospy.loginfo("wind_direction : %s", wind_direction)
	t = time.time()
	vect_temps = np.array([vect_temps[1],t,(t-vect_temps[1])])
	vect_wind_direction = np.array([vect_wind_direction[1],wind_direction,sawtooth(wind_direction-vect_wind_direction[1])])
	get = 1

def sub_wind_speed(data): # Float32
	#rospy.loginfo("[{}] I see wind speed".format(node_name))
	global wind_speed
	if data.data < 100:
		wind_speed = data.data

def sub_euler_angles(data): # Vector3
	global theta
	theta = data.x

def sub_boat_speed(data): #GPSFix
	global boat_speed, boat_timer
	boat_speed = data.speed

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
#      Main
##############################################################################################

vect_wind_direction = np.array([0,0,0])
get = 0
vect_temps = np.array([0,0,0]) # t_prec, t_curr, dt
theta = 0
wind_speed = 0
boat_speed = 0

if __name__ == '__main__':
	# kalman : https://pdfs.semanticscholar.org/7274/0405dcf42373fd4ed6fbd0a6f2d8d2208590.pdf
	node_name = 'filtre_wind_direction'
	rospy.init_node(node_name)

	rospy.Subscriber("filter_send_euler_angles", Vector3, sub_euler_angles)
	rospy.Subscriber("ardu_send_wind_direction", Float32, sub_wind_direction)
	rospy.Subscriber("ardu_send_wind_speed", Float32, sub_wind_speed)
	pub_send_wind_direction = rospy.Publisher('filter_send_wind_direction', Float32, queue_size=10)
	pub_send_wind_speed = rospy.Publisher('filter_send_wind_speed', Float32, queue_size=10)
	wind_direction_msg = Float32()
	wind_speed_msg = Float32()

	rospy.sleep(1)

	P0 = 10*np.eye(3)
	Q = 0.028**2*np.eye(3)#0.028
	R = 0.01*np.eye(3)
	EKF_yaw   = Extended_kalman_filter(np.zeros((3,1)),P0,f,F,h,H,Q,R)
	

	while not rospy.is_shutdown():
		t0 = time.time()

		if get == 1: #each time we have a new value
			get = 0
			
			z = np.array([[np.cos(vect_wind_direction[1])],[np.sin(vect_wind_direction[1])],[1]])
			[x,P] = EKF_yaw.EKF_step(vect_wind_direction[2],z)
			wind_direction = np.arctan2(x[1,0],x[0,0])

			apparent_wind = np.array([[np.cos(theta + wind_direction)],[np.sin(theta + wind_direction)]])*wind_speed
			boat_wind     = np.array([[np.cos(theta)],[np.sin(theta)]])*boat_speed
			true_wind     = apparent_wind-boat_wind
			true_wind     = np.arctan2(true_wind[1,0],true_wind[0,0])

			wind_direction_msg.data = theta + wind_direction
			pub_send_wind_direction.publish(wind_direction_msg)
			wind_speed_msg.data = wind_speed
			pub_send_wind_speed.publish(wind_speed_msg)
			rospy.loginfo("[{}] Wind direction : {}, Wind speed : {} ".format(node_name, true_wind, wind_speed))

		t1 = time.time()
		pause = vect_temps[2]/2-(t1-t0)
		#rospy.loginfo("[{}] Pause : {}".format(node_name,pause))
		rospy.sleep(0.05) #pause
