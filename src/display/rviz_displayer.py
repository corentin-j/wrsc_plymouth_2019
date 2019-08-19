#!/usr/bin/env python2

import rospy
import utm
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
import tf
import time
import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from gps_common.msg import GPSFix

##############################################################################################
#      Rviz
##############################################################################################

class Marker_rviz():

	def __init__(self,name,position=(0,0,0), angles=(0,0,0), scale=(1,1,1),m_type=1,color=(1.0,1.0,1.0)):
		"""
		name : string, name of the marker
		type : int, type of the marker
		position : tuple, (x,y,z)
		angles : tuple, (roll,pitch,yaw)
		scale : tuple, (x,y,z)
		"""
		self.marker = Marker()
		self.marker.header.frame_id = name
		self.marker.header.stamp = rospy.get_rostime()
		self.marker.ns = 'ns_'+name
		self.marker.id = 0
		self.marker.action = 0
		self.marker.pose.position.x = position[0]
		self.marker.pose.position.y = position[1]
		self.marker.pose.position.z = position[2]
		q = quaternion_from_euler(angles[0],angles[1],angles[2])
		self.marker.pose.orientation.x = q[0]
		self.marker.pose.orientation.y = q[1]
		self.marker.pose.orientation.z = q[2]
		self.marker.pose.orientation.w = q[3]
		self.marker.scale.x = scale[0]
		self.marker.scale.y = scale[1]
		self.marker.scale.z = scale[2]
		self.marker.color.r = color[0]
		self.marker.color.g = color[1]
		self.marker.color.b = color[2]
		self.marker.color.a = 1.0

		self.marker.type = m_type
		if m_type == 10:
			self.marker.mesh_resource = "package://wrsc_plymouth_jegat/meshs/"+name+".STL"

		self.pub_send_marker = rospy.Publisher('forrviz_send_'+name, Marker, queue_size=10)

	def publish(self):
		self.marker.header.stamp = rospy.get_rostime()
		self.pub_send_marker.publish(self.marker)



##############################################################################################
#      ROS
##############################################################################################

def sub_euler_angles(data):
	global yaw,pitch,roll
	yaw = data.x
	pitch = data.y
	roll = data.z

#def sub_xy(data):
#	global x, y, delta_s
#	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
#	x = data.x
#	y = data.y
#	delta_s = data.z

def sub_wind_direction(data):
	global psi
	psi = data.data

def sub_wind_force(data):
	global awind
	awind = data.data

def sub_uq(data):
	global dr
	#rospy.loginfo("U_deltar : %s, U_deltamax : %s, Q : %s",data.x, data.y, data.z)
	dr = data.x

def sub_gps_origin(data): # Vector3
	global lat_lon_origin
	lat_lon_origin[0] = [data.x, data.y]
	lat_lon_origin[1] = utm.from_latlon(data.x, data.y)

def sub_gps(data): # Vector3
	global x, y
	res = utm.from_latlon(data.latitude, data.longitude)

	x = lat_lon_origin[1][1]-res[1]
	y = lat_lon_origin[1][0]-res[0]

##############################################################################################
#      Main
##############################################################################################

if __name__ == "__main__":
	yaw,pitch,roll = 0,0,0
	#x,y = 0,0
	awind, psi = 1,-np.pi
	delta_s, dr = 0,0
	x, y = 0, 0
	lat_lon_origin = [[],[]]
	rospy.init_node('rviz_displayer')
	rospy.Subscriber("simu_send_theta", Vector3, sub_euler_angles)
	#rospy.Subscriber("simu_send_xy", Point, sub_xy)
	rospy.Subscriber("simu_send_wind_direction", Float32, sub_wind_direction)
	rospy.Subscriber("simu_send_wind_force", Float32, sub_wind_force)
	rospy.Subscriber("control_send_uq", Point, sub_uq)
	rospy.Subscriber("launch_send_gps_origin", Vector3, sub_gps_origin)
	rospy.Subscriber("simu_send_gps", GPSFix, sub_gps)

	marker_boat   = Marker_rviz("boat",(-0.5,-0.24,-0.2),(np.pi/2, 0, np.pi/2),(0.0002,0.0002,0.0002),10,(0.9,0.08,0))
	marker_rudder = Marker_rviz("rudder",(0.15,0,-0.2),(np.pi/2, 0, -np.pi/2),(0.0004,0.0004,0.0004),10)
	marker_sail   = Marker_rviz("sail",(0.55,0,0),(np.pi/2, 0, -np.pi/2),(0.0002,0.0002,0.0002),10,(0.8,0.64,0.64))
	marker_wind   = Marker_rviz("wind",(0,0,0),(0, 0, 0),(awind+0.01,0.1,0.1),0)

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		marker_boat.publish()
		marker_rudder.publish()
		marker_sail.publish()
		marker_wind.publish()
		marker_line.publish()
		rate.sleep()

		br_boat = tf.TransformBroadcaster()
		br_boat.sendTransform((x, y, 0.0),quaternion_from_euler(roll,pitch,yaw),rospy.Time.now(),"boat","map")
		br_wind = tf.TransformBroadcaster()
		br_wind.sendTransform((x+2,y,0),(quaternion_from_euler(0,0,psi)),rospy.Time.now(),"wind","map")
		br_rudder = tf.TransformBroadcaster()
		br_rudder.sendTransform((-0.5,0,0),(quaternion_from_euler(0,0,np.pi+dr)),rospy.Time.now(),"rudder","boat")
		br_sail = tf.TransformBroadcaster()
		br_sail.sendTransform((0.1,0,0.3),(quaternion_from_euler(0,0,np.pi+delta_s)),rospy.Time.now(),"sail","boat")
		br_line = tf.TransformBroadcaster()
		br_line.sendTransform((0,0,0),(quaternion_from_euler(0,0,0)),rospy.Time.now(),"line","map")