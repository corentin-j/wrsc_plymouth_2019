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
from geometry_msgs.msg import Quaternion
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

	# if m_type == 4
	def set_points(self,set_p1,set_p2):
		global lat_lon_origin
		p1 = Point()
		p1.x,p1.y = set_p1 #-75,40
		self.marker.points.append(p1)
		p2 = Point()
		p2.x,p2.y = set_p2 # 175,-40
		self.marker.points.append(p2)

##############################################################################################
#      ROS
##############################################################################################

def sub_euler_angles(data):
	global yaw,pitch,roll
	yaw = data.x
	pitch = data.y
	roll = data.z

def sub_xy(data):
	global x, y, delta_s
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

def sub_u_rudder(data):
	global dr
	#rospy.loginfo("U_deltar : %s, U_deltamax : %s, Q : %s",data.x, data.y, data.z)
	dr = data.data

def sub_u_sail(data):
	global delta_s
	#rospy.loginfo("U_deltar : %s, U_deltamax : %s, Q : %s",data.x, data.y, data.z)
	delta_s = data.data

def sub_gps_origin(data): # Vector3
	global lat_lon_origin
	lat_lon_origin[0] = [data.x, data.y]
	lat_lon_origin[1] = utm.from_latlon(data.x, data.y)

def sub_gps(data): # Vector3
	global x, y
	res = utm.from_latlon(data.latitude, data.longitude)

	x = -(lat_lon_origin[1][1]-res[1])
	y = lat_lon_origin[1][0]-res[0]

def sub_lines_to_follow(data): # Quaternion
	global pline

	res = utm.from_latlon(data.x, data.y)
	pline[0][0] = -(lat_lon_origin[1][1]-res[1])
	pline[0][1] = (lat_lon_origin[1][0]-res[0])

	res = utm.from_latlon(data.z, data.w)
	pline[1][0] = -(lat_lon_origin[1][1]-res[1])
	pline[1][1] = (lat_lon_origin[1][0]-res[0])

def sub_zone_to_stay(data): # Vector3
	global zone_to_stay

	res = utm.from_latlon(data.x, data.y)
	zone_to_stay[0] = -(lat_lon_origin[1][1]-res[1])
	zone_to_stay[1] = (lat_lon_origin[1][0]-res[0])


##############################################################################################
#      Main
##############################################################################################

if __name__ == "__main__":
	yaw,pitch,roll = 0,0,0
	pline = [[0,0],[1,1]]
	awind, psi = 1,-np.pi
	delta_s, dr = 0,0
	x, y = 0, 0
	lat_lon_origin = [[],[]]
	zone_to_stay = [0,0]
	node_name = 'rviz_displayer_line_following'
	rospy.init_node(node_name)
	rospy.Subscriber("simu_send_theta", Vector3, sub_euler_angles)
	#rospy.Subscriber("simu_send_xy", Point, sub_xy)
	rospy.Subscriber("simu_send_gps", GPSFix, sub_gps)
	rospy.Subscriber("simu_send_wind_direction", Float32, sub_wind_direction)
	rospy.Subscriber("simu_send_wind_force", Float32, sub_wind_force)
	rospy.Subscriber("control_send_u_rudder", Float32, sub_u_rudder)
	rospy.Subscriber("control_send_u_sail", Float32, sub_u_sail)
	rospy.Subscriber("launch_send_gps_origin", Vector3, sub_gps_origin)
	rospy.Subscriber("control_send_lines_to_follow", Quaternion, sub_lines_to_follow)
	rospy.Subscriber("control_send_zone_to_stay", Vector3, sub_zone_to_stay)

	marker_boat   = Marker_rviz("boat",(-0.5,-0.24,-0.2),(np.pi/2, 0, np.pi/2),(0.0002,0.0002,0.0002),10,(0.9,0.08,0))
	marker_rudder = Marker_rviz("rudder",(0.15,0,-0.2),(np.pi/2, 0, -np.pi/2),(0.0004,0.0004,0.0004),10)
	marker_sail   = Marker_rviz("sail",(0.55,0,0),(np.pi/2, 0, -np.pi/2),(0.0002,0.0002,0.0002),10,(0.8,0.64,0.64))
	marker_wind   = Marker_rviz("wind",(0,0,0),(0, 0, 0),(awind+0.01,0.1,0.1),0)
	marker_line   = Marker_rviz("line",(0,0,0),(0, 0, 0),(.2,0,0),4,(0,1,1))

	marker_zone   = Marker_rviz("zone",(0,0,-0.2),(0, 0, 0),(30,30,0.1),3,(0,0.5,0))

	while lat_lon_origin == [[],[]] and not rospy.is_shutdown():
		rospy.sleep(0.5)
		rospy.loginfo("[{}] Waiting GPS origin".format(node_name))
	rospy.loginfo("[{}] Got GPS origin {}".format(node_name,lat_lon_origin))
	

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		marker_boat.publish()
		marker_rudder.publish()
		marker_sail.publish()
		marker_wind.publish()
		marker_line.publish()
		marker_zone.publish()
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
		br_zone = tf.TransformBroadcaster()
		br_zone.sendTransform((zone_to_stay[0],zone_to_stay[1],0),(quaternion_from_euler(0,0,0)),rospy.Time.now(),"zone","map")

		marker_line.set_points(pline[0],pline[1])