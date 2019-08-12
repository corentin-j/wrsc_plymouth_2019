#!/usr/bin/env python2
from numpy import *
import numpy as np
from numpy import cos,sin
import utm

import sys
import rospkg
rospack = rospkg.RosPack()
pkg = rospack.get_path('wrsc_plymouth_jegat')
sys.path.append(pkg+'/src/my_libs')
import control_lib as cl

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
#from geometry_msgs.msg import Point
from gps_common.msg import GPSFix
from geometry_msgs.msg import Quaternion

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

##############################################################################################
#      ROS
##############################################################################################

def sub_xy(data):
	global pos_x, pos_y
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	pos_x = data.x
	pos_y = data.y

def sub_gps_origin(data): # Vector3
    global lat_lon_origin
    lat_lon_origin[0] = [data.x, data.y]
    lat_lon_origin[1] = utm.from_latlon(data.x, data.y)

def sub_gps(data): # Vector3
	global pos_x, pos_y
	res = utm.from_latlon(data.latitude, data.longitude)

	pos_x = -(lat_lon_origin[1][1]-res[1])
	pos_y = lat_lon_origin[1][0]-res[0]

def sub_wind_direction(data):
	global psi
	psi = data.data

def sub_wind_force(data):
	global awind
	awind = data.data

def sub_theta(data):
	global theta
	#rospy.loginfo("Imu heading %s ",data.orientation.x)
	theta = data.x

def sub_zone_to_stay(data): # Vector3
	global zone_to_stay

	res = utm.from_latlon(data.x, data.y)
	zone_to_stay[0] = -(lat_lon_origin[1][1]-res[1])
	zone_to_stay[1] = (lat_lon_origin[1][0]-res[0])
	print(zone_to_stay)


##############################################################################################
#      Main
##############################################################################################

pos_x, pos_y= 0, 0
awind,psi = 0, 0
theta = 0
lat_lon_origin = [[0,0],[0,0]]
zone_to_stay = [0,0]
u = [0,0]

if __name__ == '__main__':

	node_name = 'controler_station_keeping'
	rospy.init_node(node_name)
	rospy.Subscriber("simu_send_theta", Vector3, sub_theta)
	#rospy.Subscriber("simu_send_xy", Point, sub_xy)
	rospy.Subscriber("simu_send_wind_direction", Float32, sub_wind_direction)
	rospy.Subscriber("simu_send_wind_force", Float32, sub_wind_force)
	rospy.Subscriber("launch_send_gps_origin", Vector3, sub_gps_origin)
	rospy.Subscriber("simu_send_gps", GPSFix, sub_gps)
	rospy.Subscriber("control_send_zone_to_stay", Vector3, sub_zone_to_stay)

	pub_send_u_rudder = rospy.Publisher('control_send_u_rudder', Float32, queue_size=10)
	pub_send_u_sail   = rospy.Publisher('control_send_u_sail', Float32, queue_size=10)
	u_rudder_msg = Float32()
	u_sail_msg   = Float32()

	while lat_lon_origin == [[],[]] and not rospy.is_shutdown():
		rospy.sleep(0.5)
		rospy.loginfo("[{}] Waiting GPS origin".format(node_name))
	rospy.loginfo("[{}] Got GPS origin {}".format(node_name,lat_lon_origin))
	while zone_to_stay == [0,0] and not rospy.is_shutdown():
		rospy.sleep(0.5)
		rospy.loginfo("[{}] Waiting zone_to_stay {}".format(node_name, zone_to_stay))
	rospy.loginfo("[{}] Zone_to_stay {}".format(node_name,zone_to_stay))
	
	zone = 1

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		x = array([[pos_x,pos_y,theta]]).T
		
		fut_x = x + 1*array([[np.cos(theta),np.sin(theta),0]]).T

		params = [zone_to_stay[0],zone_to_stay[1],psi,5,3]
		thetabar, zone, display = cl.get_thetabar_station_keeping(params,x,zone)
		u = cl.control_station_keeping(thetabar,fut_x,psi)
		rospy.loginfo("[{}] Zone {}, {}".format(node_name,zone,thetabar))

		u_rudder_msg.data = u[0,0]
		u_sail_msg.data = u[1,0]
		pub_send_u_rudder.publish(u_rudder_msg)
		pub_send_u_sail.publish(u_sail_msg)
		rate.sleep()
