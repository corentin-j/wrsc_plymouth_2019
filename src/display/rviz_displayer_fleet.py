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
			self.marker.mesh_resource = "package://wrsc_plymouth_jegat/meshs/"+name[:-2]+".STL"

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

	def set_text(self, text):
		self.marker.text = text

##############################################################################################
#      Boat
##############################################################################################

class Boat():

	def __init__(self,num,lat_lon_origin):
		self.num = num
		self.lat_lon_origin = lat_lon_origin
		self.sub_gps = rospy.Subscriber("xbee_send_gps_"+str(num), GPSFix, self.sub_gps)
		self.sub_euler_angles = rospy.Subscriber("xbee_send_euler_angles_"+str(num), Vector3, self.sub_euler_angles)
		self.yaw = 0
		self.pitch = 0
		self.roll = 0
		self.x = 0
		self.y = 0
		self.scale = 1

	def sub_euler_angles(self,data):
		self.yaw = data.x
		self.pitch = data.y
		self.roll = data.z

	def sub_gps(self,data): # Vector3
		res = utm.from_latlon(data.latitude, data.longitude)
		self.x = -(self.lat_lon_origin[1][1]-res[1])
		self.y = self.lat_lon_origin[1][0]-res[0]

	def print_euler_angles(self):
		return "Boat {} : Yaw={}, Pitch={}, Roll={}".format(self.num,self.yaw,self.pitch,self.roll)

	def markers(self,scale = 1):
		self.scale = scale
		self.marker_boat   = Marker_rviz("boat_"+str(self.num),(-0.5*scale,-0.24*scale,-0.2*scale),(np.pi/2, 0, np.pi/2),(0.0002*scale,0.0002*scale,0.0002*scale),10,(0.9,0.08,0))
		self.marker_rudder = Marker_rviz("rudder_"+str(self.num),(0.15*scale,0,-0.2*scale),(np.pi/2, 0, -np.pi/2),(0.0004*scale,0.0004*scale,0.0004*scale),10)
		self.marker_sail   = Marker_rviz("sail_"+str(self.num),(0.55*scale,0,0),(np.pi/2, 0, -np.pi/2),(0.0002*scale,0.0002*scale,0.0002*scale),10,(0.8,0.64,0.64))
		self.marker_text   = Marker_rviz("text_"+str(self.num),(0,0,0),(0, 0, 0),(1,1,0.5*scale),9,(1,1,1))
		self.marker_text.set_text("boat_"+str(self.num))

	def transform(self):
		self.br_boat   = tf.TransformBroadcaster()
		self.br_rudder = tf.TransformBroadcaster()
		self.br_sail   = tf.TransformBroadcaster()
		self.br_text   = tf.TransformBroadcaster()

	def publish_markers(self):
		self.marker_boat.publish()
		self.marker_rudder.publish()
		self.marker_sail.publish()
		self.marker_text.publish()

	def publish_transform(self):
		self.br_boat.sendTransform((self.x, self.y, 0.0),quaternion_from_euler(self.roll,self.pitch,self.yaw),rospy.Time.now(),"boat_"+str(self.num),"map")
		self.br_rudder.sendTransform((-0.5*self.scale,0,0),(quaternion_from_euler(0,0,np.pi)),rospy.Time.now(),"rudder_"+str(self.num),"boat_"+str(self.num))
		self.br_sail.sendTransform((0.1*self.scale,0,0.3*self.scale),(quaternion_from_euler(0,0,np.pi)),rospy.Time.now(),"sail_"+str(self.num),"boat_"+str(self.num))
		self.br_text.sendTransform((0,0,1*self.scale),(quaternion_from_euler(0,0,0)),rospy.Time.now(),"text_"+str(self.num),"boat_"+str(self.num))



##############################################################################################
#      Main
##############################################################################################

def sub_gps_origin(data):
	global lat_lon_origin
	lat_lon_origin[0] = [data.x, data.y]
	lat_lon_origin[1] = utm.from_latlon(data.x, data.y)

if __name__ == "__main__":
	lat_lon_origin = [[],[]]
	nb_boats = 1
	node_name = "rviz_displayer_fleet"
	rospy.init_node(node_name)

	rospy.Subscriber("launch_send_gps_origin", Vector3, sub_gps_origin)
	while lat_lon_origin == [[],[]] and not rospy.is_shutdown():
		rospy.sleep(0.5)
		rospy.loginfo("[{}] Waiting GPS origin".format(node_name))
	rospy.loginfo("[{}] Got GPS origin {}".format(node_name,lat_lon_origin))

	l_boats = []
	for i in range(1,nb_boats+1):
		l_boats.append(Boat(i,lat_lon_origin))
	for i in l_boats:
		i.markers(6)
		i.transform()

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		for i in l_boats:
			i.publish_markers()
			i.publish_transform()

		rate.sleep()
		