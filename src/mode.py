#!/usr/bin/env python2
import numpy as np

import rospy
from std_msgs.msg import Float32

##############################################################################################
#      ROS
##############################################################################################

def sub_xbee_mode(data): # Float32
	global mode
	mode = data.data

def sub_control_u_rudder(data):
	global u_rudder, publish
	if not(mode):
		u_rudder = data.data
		publish[0] = 1

def sub_control_u_sail(data):
	global u_sail, publish
	if not(mode):
		u_sail = data.data
		publish[1] = 1

def sub_xbee_u_rudder(data):
	global u_rudder, publish
	if mode:
		u_rudder = data.data
		publish[0] = 1

def sub_xbee_u_sail(data):
	global u_sail, publish
	if mode:
		u_sail = data.data
		publish[1] = 1


##############################################################################################
#      Main
##############################################################################################

if __name__ == '__main__':

	mode = 0 # 0=controler, 1=xbee
	u_rudder, u_sail = 0, 0
	publish = [0,0] # publish rudder, publish sail

	rospy.init_node('mode')
	pub_send_u_rudder = rospy.Publisher('mode_send_u_rudder', Float32, queue_size=10)
	pub_send_u_sail   = rospy.Publisher('mode_send_u_sail', Float32, queue_size=10)

	rospy.Subscriber("control_send_u_rudder", Float32, sub_control_u_rudder)
	rospy.Subscriber("control_send_u_sail", Float32, sub_control_u_sail)
	rospy.Subscriber("xbee_send_u_rudder", Float32, sub_xbee_u_rudder)
	rospy.Subscriber("xbee_send_u_sail", Float32, sub_xbee_u_sail)
	rospy.Subscriber("xbee_send_mode", Float32, sub_xbee_mode)

	u_rudder_msg = Float32()
	u_sail_msg = Float32()

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():

		if publish[0]:
			publish[0] = 0
			u_rudder_msg.data = u_rudder
			pub_send_u_rudder.publish(u_rudder_msg)
		if publish[1]:
			publish[1] = 0
			u_sail_msg.data = u_sail
			pub_send_u_sail.publish(u_sail_msg)
		rate.sleep()
