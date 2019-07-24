#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Quaternion
from gps_common.msg import GPSFix


def sub_gps(data):
	global boat_lat, boat_long
	boat_lat = data.latitude
	boat_long = data.longitude


##############################################################################################
#      Main
##############################################################################################

boat_lat, boat_long = 50.0,-4.0

if __name__ == '__main__':

	node_name = "lines_to_follow"
	rospy.init_node(node_name)

	rospy.Subscriber("simu_send_gps", GPSFix, sub_gps)
	pub_send_lines_to_follow = rospy.Publisher('control_send_lines_to_follow', Quaternion, queue_size=10)
	lines_to_follow_msg = Quaternion()

	rate = rospy.Rate(10)
	triangle = [[50.695396,-4.236313],[50.696210,-4.235734],[50.695266,-4.235562]] #en deuxieme pour lisser [50.696190,-4.236122]
	n = len(triangle)
	point = 0
	while not rospy.is_shutdown():
		# 50.695396 -4.236313
		# 50.696210 -4.235734
		# 50.695266 -4.235562
		lines_to_follow_msg.x = triangle[point%n][0]
		lines_to_follow_msg.y = triangle[point%n][1]
		
		lines_to_follow_msg.z = triangle[(point+1)%n][0]
		lines_to_follow_msg.w = triangle[(point+1)%n][1]
		
		pub_send_lines_to_follow.publish(lines_to_follow_msg)
		dist = (boat_lat-triangle[(point+1)%n][0])**2+(boat_long-triangle[(point+1)%n][1])**2
		if dist<10**-9:
			point+=1
		#rospy.loginfo("[{}] lat:{}, lon:{}".format(node_name,dist,point))

		rate.sleep()