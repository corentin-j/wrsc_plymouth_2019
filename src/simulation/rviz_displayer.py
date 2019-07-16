#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
import tf
import time
import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point

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
			self.marker.mesh_resource = "package://WRSC_plymouth_JEGAT/meshs/"+name+".STL"
		if m_type == 4:
			p1 = Point()
			p1.x,p1.y = -75,40
			self.marker.points.append(p1)
			p2 = Point()
			p2.x,p2.y = 175,-40
			self.marker.points.append(p2)

		self.pub_send_marker = rospy.Publisher('forrviz_send_'+name, Marker, queue_size=10)

	def publish(self):
		self.marker.header.stamp = rospy.get_rostime()
		self.pub_send_marker.publish(self.marker)

def sub_euler_angles(data):
	global yaw,pitch,roll
	yaw = data.x
	pitch = data.y
	roll = data.z

def sub_xy(data):
	global x, y, delta_s
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	x = data.x
	y = data.y
	delta_s = data.z

def sub_wind(data):
	global awind, psi
	#rospy.loginfo("Awind : %s, Psi : %s",data.x, data.y)
	awind = data.x
	psi = data.y

def sub_uq(data):
    global dr
    #rospy.loginfo("U_deltar : %s, U_deltamax : %s, Q : %s",data.x, data.y, data.z)
    dr = data.x

if __name__ == "__main__":
	yaw,pitch,roll = 0,0,0
	x,y = 0,0
	awind, psi = 1,-np.pi
	delta_s, dr = 0,0
	rospy.init_node('rviz_displayer')
	rospy.Subscriber("simu_send_theta", Vector3, sub_euler_angles)
	rospy.Subscriber("simu_send_xy", Point, sub_xy)
	rospy.Subscriber("simu_send_wind", Point, sub_wind)
	rospy.Subscriber("control_send_uq", Point, sub_uq)

	marker_boat   = Marker_rviz("boat",(-0.5,-0.24,-0.2),(np.pi/2, 0, np.pi/2),(0.0002,0.0002,0.0002),10,(0.9,0.08,0))
	marker_rudder = Marker_rviz("rudder",(0.15,0,-0.2),(np.pi/2, 0, -np.pi/2),(0.0004,0.0004,0.0004),10)
	marker_sail   = Marker_rviz("sail",(0.55,0,0),(np.pi/2, 0, -np.pi/2),(0.0002,0.0002,0.0002),10,(0.8,0.64,0.64))
	marker_wind   = Marker_rviz("wind",(0,0,0),(0, 0, 0),(awind+0.01,0.1,0.1),0)
	marker_line   = Marker_rviz("line",(0,0,0),(0, 0, 0),(.2,0,0),4,(0,1,1))

	a = 0
	while not rospy.is_shutdown():
		marker_boat.publish()
		marker_rudder.publish()
		marker_sail.publish()
		marker_wind.publish()
		marker_line.publish()
		time.sleep(0.1)

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