#!/usr/bin/env python2

import numpy as np
from numpy import cos,sin
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy import optimize

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import time
import os

from filter_lib import *

def rotation_matrix(pitch, roll):

	R_pitch  = np.array([[cos(pitch),0,-sin(pitch)],[0,1,0],[sin(pitch),0,cos(pitch)]])
	R_roll   = np.array([[1,0,0],[0,cos(roll),sin(roll)],[0,-sin(roll),cos(roll)]])
	R_repere = np.array([[0,-1,0],[-1,0,0],[0,0,1]])

	return np.dot(R_repere,np.dot(R_pitch,R_roll))


##############################################################################################
#      ROS
##############################################################################################

# http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
def sub_imu_nf(data):
	global vect_imu, vect_temps, get
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	vect_imu[0,0] = data.linear_acceleration.x # ax
	vect_imu[1,0] = data.linear_acceleration.y # ay
	vect_imu[2,0] = data.linear_acceleration.z # az
	vect_imu[3,0] = data.angular_velocity.x    # gx
	vect_imu[4,0] = data.angular_velocity.y    # gy
	vect_imu[5,0] = data.angular_velocity.z    # gz
	t = data.header.stamp.nsecs
	vect_temps = np.array([vect_temps[1],t,(t-vect_temps[1])/1000.])
	get = 1

def sub_mag_nf(data):
	global vect_imu
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	vect_imu[6,0] = data.magnetic_field.x         # mx
	vect_imu[7,0] = data.magnetic_field.y         # my
	vect_imu[8,0] = data.magnetic_field.z         # mz


##############################################################################################
#      Calibration
##############################################################################################

def resSphere(p,x,y,z):
	""" residuals from sphere fit """

	a,b,c,r = p                             # a,b,c are center x,y,c coords to be fit, r is the radius to be fit
	distance = np.sqrt( (x-a)**2 + (y-b)**2 + (z-c)**2 )
	err = distance - r                 # err is distance from input point to current fitted surface

	return err

def calibrate_imu(n = 200):# number of points to calibrate

	# --- Get data ---------------------------------------------------------- #
	mx,my,mz = np.zeros(n), np.zeros(n), np.zeros(n)
	counter = 0
	while counter < n: # while there is not enough values
		a = vect_imu.copy()
		rospy.sleep(0.01)
		if not(np.all(a==vect_imu)): #each time we have a new value
			mx[counter],my[counter],mz[counter] = a[6,0],a[7,0],a[8,0] # the value is added
			counter += 1
			rospy.loginfo(counter)

	params = [0.,0.,0.,0.]
	myResult = optimize.leastsq(resSphere, params, args=(mx,my,mz) )
	ox, oy, oz, r = myResult[0]
	rospy.loginfo(myResult[0])

	# ---- Save in text file ------------------------------------------------- #
	THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
	my_file = os.path.join(THIS_FOLDER, 'imu_previous_calibration.txt')
	f = open(my_file,'w')
	#f.write('offset_x:{}\noffset_y:{}\noffset_z:{}'.format(np.mean(mx), np.mean(my), np.mean(mz)))
	f.write('offset_x:{}\noffset_y:{}\noffset_z:{}'.format(-ox,-oy,-oz))
	f.close()
	offset = np.zeros((9,1))
	offset[6,0],offset[7,0],offset[8,0] = -ox,-oy,-oz
	return offset

def get_previous_imu():
	THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
	my_file = os.path.join(THIS_FOLDER, 'imu_previous_calibration.txt')
	f = open(my_file,'r')
	doc = f.readlines()
	f.close()
	offset = np.zeros((9,1))
	for i in range(len(doc)):
		offset[i+6] = float(doc[i].split(':')[1])
	rospy.loginfo(offset)
	return offset

##############################################################################################
#      Filtre
##############################################################################################

# ----  EKF plus dur --------------------------------------------------------- #

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

def EKF_predict(x_prec,P_prec,u,f,F):
	Fx = F(x_prec,u)
	x_pred = f(x_prec,u)
	P_pred = np.matmul(np.matmul(Fx,P_prec),np.transpose(Fx))+Q
	return(x_pred,P_pred)

def EKF_update(z,x_pred,P_pred,h,H):
	Hk = H(x_pred)
	#innovation
	y = z - h(x_pred)
	S = np.matmul(np.matmul(Hk,P_pred),np.transpose(Hk))+R
	# Kalman gain
	K = np.matmul(np.matmul(P_pred,np.transpose(Hk)),np.linalg.inv(S))
	# Update
	x = x_pred + np.matmul(K,y)
	I = np.eye(P_pred.shape[0])
	P = np.matmul((I-np.matmul(K,Hk)), P_pred)
	
	return [x,P]


##############################################################################################
#      Display
##############################################################################################

def clear(ax):
    plt.pause(0.001)
    plt.cla()
    a = 15
    ax.set_xlim(-a,a)
    ax.set_ylim(-a,a)
    ax.set_zlim(-a,a)

def display_mag(offset, scale=1):

	fig = plt.figure()
	ax = plt.axes(projection='3d')
	l = []

	t0 = time.time()
	while not rospy.is_shutdown():
		m = (vect_imu + offset)*scale
		mx,my,mz = m[6,0],m[7,0],m[8,0]
		l.append([mx,my,mz])
		clear(ax)
		ax.plot([0,mx],[0,my],[0,mz],'r')
		ax.plot([0,mx],[0,0],[0,0],'b')
		ax.plot([mx,mx],[0,my],[0,0],'b')
		ax.plot([mx,mx],[my,my],[0,mz],'b')
		ax.plot([0,10],[0,0],[0,0],'g')
		ax.plot([0,0],[0,5],[0,0],'g')
		ax.plot([0,0],[0,0],[0,5],'g')
		for i in l:
			ax.scatter(i[0],i[1],i[2])
		#rospy.loginfo(np.arctan2(my,mx)*180/np.pi)
		rospy.loginfo(np.sqrt(mx**2+my**2+mz**2))

##############################################################################################
#      Main
##############################################################################################

vect_imu = np.zeros((9,1)) # ax;ay;az;gx;gy;gz;mx;my;mz
vect_temps = np.array([0,0,0]) # t_prec, t_curr, dt
get = 0

if __name__ == '__main__':
	# kalman : https://pdfs.semanticscholar.org/7274/0405dcf42373fd4ed6fbd0a6f2d8d2208590.pdf

	try:
		# Three calibration modes : 
		# 'None' for no calibration, 'New' for new calibration, 'Previous' to take the previous one
		calibration = 'Previous'
		rospy.init_node('filtre_imu')
		rospy.Subscriber("ardu_send_imu", Imu, sub_imu_nf)
		rospy.Subscriber("ardu_send_mag", MagneticField, sub_mag_nf)
		if calibration == 'Previous':
			offset = get_previous_imu()
		elif calibration == 'New':
			offset = calibrate_imu()
		else:

			offset = np.zeros((9,1))

		raw_imu = vect_imu + offset
		low_pass_imu = raw_imu.copy()

		#display_mag(offset, scale)
		alpha = 0.3

		pub_send_euler_angles = rospy.Publisher('imu_send_euler_angles', Vector3, queue_size=10)
		euler_angles_msg = Vector3()
		
		Gx_yaw = np.array([[2,0],[0,1]])
		Gx_pitch = np.array([[2,0],[0,1]])
		Gx_roll = np.array([[2,0],[0,1]])
		xhat_yaw = np.array([[0],[0]])
		xhat_pitch = np.array([[0],[0]])
		xhat_roll = np.array([[0],[0]])
		Galpha = 0.028**2*np.eye(2,2)#0.028**2
		Gbeta = 0.095*np.eye(2)# 0.095**2

		dt = vect_temps[2]		
		A = np.array([[1,-dt*raw_imu[5,0]],[dt*raw_imu[5,0],1]])
		C = np.array([[1,0],[0,1]])
		kf_yaw = Kalman_filter(xhat_yaw,Gx_yaw,A,0,C,Galpha,Gbeta)

		x_EKF_yaw = np.array([[0],[0],[0]])
		P_EKF_yaw = 10*np.eye(3)
		x_EKF_pitch = np.array([[0],[0],[0]])
		P_EKF_pitch = 10*np.eye(3)
		x_EKF_roll = np.array([[0],[0],[0]])
		P_EKF_roll = 10*np.eye(3)

		n = 5
		lx,ly = np.zeros(n),np.zeros(n)

		l_temps = []
		l_yaw_EKF = []
		l_yaw_kalman = []
		l_yaw_raw = []
		#l_pitch_EKF = []
		#l_pitch_kalman = []
		#l_pitch_raw = []
		#l_roll_EKF = []
		#l_roll_kalman = []
		#l_roll_raw = []

		C_pitch = np.arctan2(raw_imu[1,0],raw_imu[0,0])
		while not rospy.is_shutdown():
			raw_imu = (vect_imu + offset) * 1
			#display_mag(offset)
			#prev_yaw = raw_roll_pitch_yaw(new_imu, prev_yaw)
			rospy.sleep(0.01)
			a = raw_imu.copy()


			if get == 1:#not(np.all(a==vect_imu)): #each time we have a new value
				get = 0
				low_pass_imu = a#alpha*a +(1-alpha)*low_pass_imu
				ax,ay,az = low_pass_imu[0,0],low_pass_imu[1,0],low_pass_imu[2,0]
				gx,gy,gz = low_pass_imu[3,0],low_pass_imu[4,0],low_pass_imu[5,0]
				mx,my,mz = low_pass_imu[6,0],low_pass_imu[7,0],low_pass_imu[8,0]
				dt = vect_temps[2]
				#lx[:n-1],ly[:n-1] = lx[1:], ly[1:]
				#lx[n-1],ly[n-1] = mx,my
				#rospy.loginfo([np.median(l),a[7,0]])
				#rospy.loginfo([np.arctan2(np.median(ly),np.median(lx))*180/np.pi,np.arctan2(my,mx)*180/np.pi,np.arctan2(a[7,0],a[6,0])*180/np.pi])
				
				# ----  Calcul des angles ------------------------------------------------------ #

				A_pitch = np.arctan2(ay,az)
				A_roll = np.arctan2(-ax,np.sqrt(ay**2+az**2))
				Mx = mx*cos(A_pitch)+mz*sin(A_pitch) #tilt compensation
				My = mx*sin(A_roll)*sin(A_pitch)+my*cos(A_roll)-mz*sin(A_roll)*cos(A_pitch)
				G_yaw = np.arctan2(-My,Mx)
				

				# ----  Kalman Filter ---------------------------------------------------------- #

				#[xhat_yaw,Gx_yaw]     = use_kalman(xhat_yaw  ,Gx_yaw  ,G_yaw  ,-gz)
				A = np.array([[1,-dt*gz],[dt*gz,1]])
				y = np.array([[np.cos(G_yaw)],[np.sin(G_yaw)]])
				[xhat_yaw,Gx_yaw] = kf_yaw.kalman_step(0,y,A)
				#[xhat_pitch,Gx_pitch] = use_kalman(xhat_pitch,Gx_pitch,A_pitch,gx)
				#[xhat_roll,Gx_roll]   = use_kalman(xhat_roll ,Gx_roll ,A_roll ,gy)
				
				#rospy.loginfo([kalman_to_deg(xhat_yaw),kalman_to_deg(xhat_pitch),kalman_to_deg(xhat_roll)])

				#R_mat = rotation_matrix(kalman_to_rad(xhat_pitch),kalman_to_rad(xhat_roll))
				#acc_NED = np.dot(np.linalg.inv(R_mat),np.array([[ax],[ay],[az]]))
				
				Q = 0.028**2*np.eye(3)#0.028
				R = 0.1*np.eye(3)

				# ----  Extended Kalman Filter ------------------------------------------------- #

				[x_p_yaw,P_p_yaw] = EKF_predict(x_EKF_yaw,P_EKF_yaw,-gz,f,F)
				z = np.array([[np.cos(G_yaw)],[np.sin(G_yaw)],[1]])
				[x_EKF_yaw,P_EKF_yaw] = EKF_update(z,x_p_yaw,P_p_yaw,h,H)
				yaw_EKF = np.arctan2(x_EKF_yaw[1,0],x_EKF_yaw[0,0])-0.035
				#rospy.loginfo([np.arctan2(x_EKF[1,0],x_EKF[0,0])*180/np.pi,kalman_to_deg(xhat_yaw),G_yaw*180/np.pi])
				
				[x_p_pitch,P_p_pitch] = EKF_predict(x_EKF_pitch,P_EKF_pitch,gx,f,F)
				z = np.array([[np.cos(A_pitch)],[np.sin(A_pitch)],[1]])
				[x_EKF_pitch,P_EKF_pitch] = EKF_update(z,x_p_pitch,P_p_pitch,h,H)
				pitch_EKF = np.arctan2(x_EKF_pitch[1,0],x_EKF_pitch[0,0])-0.018

				[x_p_roll,P_p_roll] = EKF_predict(x_EKF_roll,P_EKF_roll,gy,f,F)
				z = np.array([[np.cos(A_roll)],[np.sin(A_roll)],[1]])
				[x_EKF_roll,P_EKF_roll] = EKF_update(z,x_p_roll,P_p_roll,h,H)
				roll_EKF = np.arctan2(x_EKF_roll[1,0],x_EKF_roll[0,0])+0.008

				v = ax**2+ay**2+az**2
				coeff = min(abs(v-1.05),0.15)/0.15
				C_pitch = (C_pitch + dt*gx)#*coeff + A_pitch*(1-coeff)
				rospy.loginfo([C_pitch,v,coeff])

				euler_angles_msg.x = yaw_EKF
				euler_angles_msg.y = pitch_EKF
				euler_angles_msg.z = roll_EKF
				pub_send_euler_angles.publish(euler_angles_msg)

				l_temps.append(vect_temps[1])
				l_yaw_raw.append(A_pitch*180/np.pi)
				l_yaw_kalman.append(C_pitch*180/np.pi)
				l_yaw_EKF.append(pitch_EKF*180/np.pi)
				#l_pitch_raw.append(A_pitch*180/np.pi)
				#l_pitch_kalman.append(kalman_to_deg(xhat_pitch))
				#l_pitch_EKF.append(pitch_EKF*180/np.pi)
				#l_roll_raw.append(A_roll*180/np.pi)
				#l_roll_kalman.append(kalman_to_deg(xhat_roll))
				#l_roll_EKF.append(roll_EKF*180/np.pi)
				
				#plt.subplot('131')
				plt.plot(l_temps,l_yaw_raw,'b')
				plt.plot(l_temps,l_yaw_kalman,'r')
				plt.plot(l_temps,l_yaw_EKF,'g')
				plt.xlim((vect_temps[1]-5000,vect_temps[1]))
				plt.ylim((-50,50))
				#plt.subplot('132')
				#plt.plot(l_temps,l_pitch_raw,'b')
				#plt.plot(l_temps,l_pitch_kalman,'r')
				#plt.plot(l_temps,l_pitch_EKF,'g')
				#plt.xlim((vect_temps[1]-5000,vect_temps[1]))
				#plt.ylim((-50,50))
				#plt.subplot('133')
				#plt.plot(l_temps,l_roll_raw,'b')
				#plt.plot(l_temps,l_roll_kalman,'r')
				#plt.plot(l_temps,l_roll_EKF,'g')
				#plt.xlim((vect_temps[1]-5000,vect_temps[1]))
				#plt.ylim((-10,10))
				plt.pause(0.01)
				plt.cla()


	except rospy.ROSInterruptException:
		pass







