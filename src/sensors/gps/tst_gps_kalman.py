#!/usr/bin/env python2
import matplotlib.pyplot as plt
import time

#import rospy
#from sensor_msgs.msg import NavSatFix
from filter_lib import *

def sub_sat_fix(data):
	global lat,lon,f
	lat.append(data.latitude)
	lon.append(data.longitude)
	f.write(str(data.latitude)+" "+str(data.longitude)+'\n')

def rkt_navi():
	f = open("f9_rtknavi_2346.pos","r")
	doc = f.readlines()
	f.close()
	lat,lon = [],[]
	for i in range(2,len(doc)):
		s = doc[i].split(' ')
		lat.append(float(s[4]))
		lon.append(float(s[5]))
	print(len(lat))

	plt.plot(lat,lon)
	plt.show()

if __name__ == '__main__':
	#lat,lon = [],[]
	#f = open("log.txt","w")
	#rospy.loginfo(f)
	#rospy.init_node('tst_gps')
	#rospy.Subscriber("fix", NavSatFix, sub_sat_fix)
	#t0 = time.time()
	#while not rospy.is_shutdown() and (time.time()-t0)<200:
	#	pass
	#plt.plot(lat,lon,'.')
	#plt.show()
	#f.close()
	x = np.array([[50],[-4]])
	G = np.array([[1,0],[0,1]])
	Q = np.eye(2)
	R = np.eye(2)
	kal = Kalman_filter(x,G,np.eye(2),0,np.eye(2),Q,R)
	f = open("log.txt","r")
	doc = f.readlines()
	f.close()
	raw_lat,raw_lon = [],[]
	kf_lat,kf_lon = [],[]
	for i in doc:
		s = i.split(' ')
		try:
			lat,lon = float(s[0]), float(s[1])
			raw_lat.append(lat)
			raw_lon.append(lon)
			y = np.array([[lat],[lon]])
			res = kal.kalman_step(0,y)
			kf_lat.append(res[0][0,0])
			kf_lon.append(res[0][1,0])
			print(res[0])
		except:
			print('invalid data')
	plt.plot(raw_lat,raw_lon,'b.')
	plt.plot(kf_lat,kf_lon,'g.')
	plt.show()



