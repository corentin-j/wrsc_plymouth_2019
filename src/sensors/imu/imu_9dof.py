#!/usr/bin/env python2

import numpy as np
from numpy import cos,sin
from scipy import optimize
import os
import time

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3

import sys
import rospkg
rospack = rospkg.RosPack()
pkg = rospack.get_path('wrsc_plymouth_jegat')
sys.path.append(pkg+'/src/my_libs')
from filter_lib import *


class Imu_9dof():

	def __init__(self,topicName_acc_gyro, topicName_mag, pubName_euler_angles):
		rospy.Subscriber(topicName_acc_gyro, Imu, self.sub_imu)
		rospy.Subscriber(topicName_mag, MagneticField, self.sub_mag)
		self.vect_agm = np.zeros((9,1))
		self.offset = np.zeros((9,1))
		self.vect_temps = np.zeros(3)
		self.get = 0
		self.yaw, self.pitch, self.roll = 1,1,1
		self.pub_send_euler_angles = rospy.Publisher(pubName_euler_angles, Vector3, queue_size=10)
		self.lp = None
		self.vect_lp = np.zeros((9,1))

	###################################################################
	#----- Subscribers -----------------------------------------------#
	###################################################################

	def sub_imu(self,data):
		self.vect_agm[0,0] = data.linear_acceleration.x # ax
		self.vect_agm[1,0] = data.linear_acceleration.y # ay
		self.vect_agm[2,0] = data.linear_acceleration.z # az
		self.vect_agm[3,0] = data.angular_velocity.x    # gx
		self.vect_agm[4,0] = data.angular_velocity.y    # gy
		self.vect_agm[5,0] = data.angular_velocity.z    # gz
		t = data.header.stamp.nsecs
		self.vect_temps = np.array([self.vect_temps[1],t,(t-self.vect_temps[1])/1000.])
		self.get = 1
		#rospy.loginfo(self.vect_agm)

	def sub_mag(self,data):
		self.vect_agm[6,0] = data.magnetic_field.x      # mx
		self.vect_agm[7,0] = data.magnetic_field.y      # my
		self.vect_agm[8,0] = data.magnetic_field.z      # mz

	###################################################################
	#----- Calibration -----------------------------------------------#
	###################################################################

	def calibration(self):
		rospy.loginfo("Imu calibration begins")
		oa = self.calibration_acceleration()
		og = self.calibration_gyro()
		om = self.calibration_magnetometer()

		THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
		my_file = os.path.join(THIS_FOLDER, 'imu_previous_calibration.txt')
		f = open(my_file,'w')
		f.write('norm_a:{}\n'.format(oa))
		f.write('offset_gx:{}\noffset_gy:{}\noffset_gz:{}\n'.format(og[0],og[1],og[2]))
		f.write('offset_mx:{}\noffset_my:{}\noffset_mz:{}'.format(om[0],om[1],om[2]))
		f.close()
		rospy.loginfo("Imu calibration ends")


	def calibration_acceleration(self, number_of_points=40):
		"""
		The norm of the vector retuned by the accelerometer should be 1g, but there is an offset.
		The calulated value is the mean of the norm of the measure acceleration vector
		"""
		rospy.loginfo("\tAccelerometers calibration begins")
		rospy.loginfo("\t\tPlease don't move")
		rospy.sleep(3)
		rospy.loginfo("\t\tReady ?")
		rospy.sleep(1)
		rospy.loginfo("\t\tGetting data")
		l = []
		counter = 0
		while not rospy.is_shutdown() and counter < number_of_points:
			if self.get == 1: #each time we have a new value
				self.get = 0
				norm_v = np.sum(self.vect_agm[:3,:]**2)
				l.append(norm_v)
				counter += 1
		rospy.loginfo("\tAccelerometers calibration ends")
		return np.mean(l)

	def calibration_gyro(self,number_of_points=200):
		"""
		Return the offset of the gryroscope
		"""
		rospy.loginfo("\tGyrometrers calibration begins")
		rospy.loginfo("\t\tPlease don't move")
		rospy.sleep(3)
		rospy.loginfo("\t\tReady ?")
		rospy.sleep(1)
		rospy.loginfo("\t\tGetting data")
		counter = 0
		lgx,lgy,lgz = [],[],[]
		while not rospy.is_shutdown() and counter < number_of_points:
			if self.get == 1: #each time we have a new value
				self.get = 0
				lgx.append(self.vect_agm[3,0])
				lgy.append(self.vect_agm[4,0])
				lgz.append(self.vect_agm[5,0])
				counter += 1
		rospy.loginfo("\t\tOffsets : {},{},{}".format(-np.mean(lgx), -np.mean(lgy), -np.mean(lgz)))
		rospy.loginfo("\tGyrometrers calibration ends")
		return -np.mean(lgx), -np.mean(lgy), -np.mean(lgz)

	def calibration_magnetometer(self,number_of_sec=60):
		"""
		Return the offset of the magnetometer
		"""
		rospy.loginfo("\tMagnetometer calibration begins")
		rospy.loginfo("\t\tPlease rotate the imu in many direction to cover the entire 3D rotations")
		rospy.sleep(3)
		rospy.loginfo("\t\tReady ?")
		rospy.sleep(1)
		rospy.loginfo("\t\tGetting data, keep turning")
		mx,my,mz = [],[],[]
		t0 = rospy.get_time()
		while not rospy.is_shutdown() and (rospy.get_time() - t0) < number_of_sec:
			if self.get == 1: #each time we have a new value
				self.get = 0
				mx.append(self.vect_agm[6,0])
				my.append(self.vect_agm[7,0])
				mz.append(self.vect_agm[8,0])
		rospy.loginfo("\t\tStop turning")

		params = [0.,0.,0.,0.]
		myResult = optimize.leastsq(self.res_sphere, params, args=(np.array(mx),np.array(my),np.array(mz)) )
		ox, oy, oz, r = myResult[0]
		rospy.loginfo("\t\tOffsets : {},{},{}".format(-ox,-oy,-oz))
		rospy.loginfo("\tMagnetometer calibration ends")
		return -ox,-oy,-oz

	def res_sphere(self,p,x,y,z):
		""" residuals from sphere fit """
		a,b,c,r = p                             # a,b,c are center x,y,c coords to be fit, r is the radius to be fit
		distance = np.sqrt( (x-a)**2 + (y-b)**2 + (z-c)**2 )
		err = distance - r                 # err is distance from input point to current fitted surface
		return err

	def get_previous_calibration(self):
		THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
		my_file = os.path.join(THIS_FOLDER, 'imu_previous_calibration.txt')
		f = open(my_file,'r')
		doc = f.readlines()
		f.close()
		offset = np.zeros((9,1))
		for i in range(1,len(doc)):
			offset[i+2] = float(doc[i].split(':')[1])
		return offset

	def get_calibration(self,your_calibration):
		if your_calibration == 0: # previous
			offset = self.get_previous_calibration()
		elif your_calibration == 1: # new
			self.calibration()
			offset = self.get_previous_calibration()
		else: # no calibration
			offset = np.zeros((9,1))
			rospy.loginfo("WARNING : No chosen calibration")
		return offset

	###################################################################
	#----- Filtre ----------------------------------------------------#
	###################################################################

	def f(self,x,u):
		dt = self.vect_temps[2]	
		mat = np.array([[1,-dt*u,0],[dt*u,1,0],[0,0,1]])
		mat = np.matmul(mat,x)
		return mat

	def F(self,x,u):
		dt = self.vect_temps[2]
		mat = np.array([[1,-dt*u,0],[dt*u,1,0],[0,0,1]])
		return mat

	def h(self,x):
		x1,x2 = x[0,0],x[1,0]
		mat = np.array([[x1],[x2],[x1**2+x2**2]])
		return mat

	def H(self,x):
		x1,x2 = x[0,0],x[1,0]
		mat = np.array([[1,0,0],[0,1,0],[2*x1,2*x2,0]])
		return mat

	def init_ekf(self):
		P0 = 10*np.eye(3)
		Q = 0.028**2*np.eye(3)#0.028
		R = 0.01*np.eye(3)

		self.ekf_yaw   = Extended_kalman_filter(np.zeros((3,1)),P0,self.f,self.F,self.h,self.H,Q,R)
		self.ekf_pitch = Extended_kalman_filter(np.zeros((3,1)),P0,self.f,self.F,self.h,self.H,Q,R)
		self.ekf_roll  = Extended_kalman_filter(np.zeros((3,1)),P0,self.f,self.F,self.h,self.H,Q,R)

	def kalman_compute(self):
		angles = self.get_angles()
		dt = self.vect_temps[2]

		z = np.array([[np.cos(angles[0])],[np.sin(angles[0])],[1]])
		[x,P] = self.ekf_yaw.EKF_step(self.vect_lp[5,0],z)
		self.yaw = np.arctan2(x[1,0],x[0,0])

		z = np.array([[np.cos(angles[1])],[np.sin(angles[1])],[1]])
		[x,P] = self.ekf_pitch.EKF_step(-self.vect_lp[3,0],z)
		self.pitch = np.arctan2(x[1,0],x[0,0])-0.018

		z = np.array([[np.cos(angles[2])],[np.sin(angles[2])],[1]])
		[x,P] = self.ekf_roll.EKF_step(-self.vect_lp[4,0],z)
		self.roll = np.arctan2(x[1,0],x[0,0])+0.008

		#rospy.loginfo("[{}] {}".format(node_name,np.array([self.yaw,self.pitch,self.roll])*180/np.pi))
		
	###################################################################
	#----- Euler angles ----------------------------------------------#
	###################################################################

	def get_angles(self):
		vect_corr = self.vect_agm + self.offset
		if self.lp == None:
			alpha = 1.0
			self.lp = Low_pass_filter(alpha,vect_corr)
		else:
			self.vect_lp = self.lp.low_pass_next(vect_corr)

		ax,ay,az = -self.vect_lp[1,0],-self.vect_lp[0,0],-self.vect_lp[2,0]
		gx,gy,gz = -self.vect_lp[4,0],-self.vect_lp[3,0],-self.vect_lp[5,0]
		mx,my,mz = -self.vect_lp[6,0],-self.vect_lp[7,0],self.vect_lp[8,0]

		# ----  Calcul des angles ------------------------------------------------------ #

		if ax != 0 and ay != 0 and az != 0:
			ax_norm = ax/np.sqrt(ax**2+ay**2+az**2)
			ay_norm = ay/np.sqrt(ax**2+ay**2+az**2)
		else:
			ax_norm, ay_norm = 0 , 0

		A_pitch = np.arcsin(ax_norm)
		A_roll = -np.arcsin(ay_norm/np.cos(A_pitch))
		Mx = mx*cos(A_pitch)+mz*sin(A_pitch)
		My = mx*sin(A_roll)*sin(A_pitch)+my*cos(A_roll)-mz*sin(A_roll)*cos(A_pitch)
		G_yaw = np.arctan2(My,Mx)
		dt = self.vect_temps[2]
		#if gz > 0.01:
		#	G_yaw =  G_yaw*0.5+0.5*(dt*gz+self.yaw)
		return G_yaw,A_pitch,A_roll

	def publish(self):
		euler_angles_msg = Vector3()
		euler_angles_msg.x = self.yaw
		euler_angles_msg.y = self.pitch
		euler_angles_msg.z = self.roll
		self.pub_send_euler_angles.publish(euler_angles_msg)



if __name__ == '__main__':
	node_name = 'imu_9dof'
	rospy.init_node(node_name)
	mode = rospy.get_param('calibration_mode',0)
	imu = Imu_9dof("ardu_send_imu", "ardu_send_mag","filter_send_euler_angles")
	imu.offset = imu.get_calibration(mode)
	imu.init_ekf()
	th = 0
	rospy.loginfo("[{}] Waiting data from Arduino".format(node_name))
	while (imu.vect_temps[2] > 1 or imu.vect_temps[2] == 0) and not rospy.is_shutdown():
		rospy.sleep(0.5)
	rospy.loginfo("[{}] Connected to Arduino, program starts".format(node_name))
	while not rospy.is_shutdown():
		t0 = time.time()
		if imu.get == 1:
			imu.get = 0
			imu.kalman_compute()
			imu.publish()
		t1 = time.time()
		pause = imu.vect_temps[2]/2-(t1-t0)
		rospy.sleep(pause)
