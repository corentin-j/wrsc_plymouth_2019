#!/usr/bin/env python2
from numpy import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import cos,sin

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point

from filter_lib import *

##############################################################################################
#      ROS
##############################################################################################

def sub_uq(data):
    global u, q
    #rospy.loginfo("U_deltar : %s, U_deltamax : %s, Q : %s",data.x, data.y, data.z)
    u = np.array([[data.x],[data.y]])
    q = data.z

##############################################################################################
#      Display
##############################################################################################

def init_figure(xmin,xmax,ymin,ymax): 
    fig = plt.figure(0)
    ax = fig.add_subplot(111, aspect='equal')   
    ax.xmin=xmin
    ax.xmax=xmax
    ax.ymin=ymin
    ax.ymax=ymax
    clear(ax)
    return ax

def clear(ax):
    plt.pause(0.001)
    plt.cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)

def draw_sailboat(x,delta_s,delta_r,psi,awind):
    x=x.flatten()
    theta=x[2]
    hull=np.array([[-1,5,7,7,5,-1,-1,-1],[-2,-2,-1,1,2,2,-2,-2],[1,1,1,1,1,1,1,1]])
    sail=np.array([[-7,0],[0,0],[1,1]])
    rudder=np.array([[-1,1],[0,0],[1,1]])
    R=np.array([[cos(theta),-sin(theta),x[0]],[sin(theta),cos(theta),x[1]],[0,0,1]])
    Rs=np.array([[cos(delta_s),-sin(delta_s),3],[sin(delta_s),cos(delta_s),0],[0,0,1]])
    Rr=np.array([[cos(delta_r),-sin(delta_r),-1],[sin(delta_r),cos(delta_r),0],[0,0,1]])
    draw_arrow(x[0]+5,x[1],psi,5*awind,'red')
    R1 = np.matmul(R, hull)
    R2 = np.matmul(np.matmul(R, Rs), sail)
    R3 = np.matmul(np.matmul(R, Rr), rudder)
    plot2D(R1,'black');       
    plot2D(R2,'red',2);       
    plot2D(R3,'red',2);

def plot2D(M,col='black',w=1):
    plt.plot(M[0, :], M[1, :], col, linewidth = w)

def draw_arrow(x,y,theta,L,col):
    e=0.2
    M1=L*np.array([[0,1,1-e,1,1-e],[0,0,-e,0,e]])
    M=np.append(M1,[[1,1,1,1,1]],axis=0)
    R=np.array([[cos(theta),-sin(theta),x],[sin(theta),cos(theta),y],[0,0,1]])
    plot2D(np.matmul(R, M),col)    

##############################################################################################
#      Controle
##############################################################################################

def controle(x, q):

	r = 5
	zeta = pi/4

	x=x.flatten()
	theta=x[2]

	m = array([[x[0]], [x[1]]])
	e = linalg.det(hstack(((b-a)/linalg.norm(b-a), m-a)))
	print(e,q)
	phi = arctan2(b[1,0]-a[1,0], b[0,0]-a[0,0])

	if abs(e) > r:
	    q = sign(e)

	thetabar = phi - arctan(e/r)

	if (cos(psi-thetabar)+cos(zeta))<0:
	    thetabar = pi+psi-zeta*q

	deltar = 2/pi*arctan(tan(0.5*(theta-thetabar)))
	deltamax = pi/4*(cos(psi-thetabar)+1)
	u = array([[deltar],[deltamax]])
	return u, q

##############################################################################################
#      ROS
##############################################################################################

def sub_xy(data):
	global pos_x, pos_y, delta_s
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	pos_x = data.x
	pos_y = data.y
	delta_s = data.z

def sub_wind(data):
	global awind, psi
	#rospy.loginfo("Awind : %s, Psi : %s",data.x, data.y)
	awind = data.x
	psi = data.y

def sub_imu(data):
	global theta
	#rospy.loginfo("Imu heading %s ",data.orientation.x)
	theta = data.orientation.x

##############################################################################################
#      Main
##############################################################################################

pos_x, pos_y = 0, 0
awind, psi = 0, 0
theta = 0
delta_s = 0

if __name__ == '__main__':
	try:
		# --- Display ---------- #

		ax=init_figure(-100,250,-100,60)
		a = array([[-75],[40]])   
		b = array([[175],[-40]])
		q = 1
		rospy.init_node('controler')
		rospy.Subscriber("simu_send_imu", Imu, sub_imu)
		rospy.Subscriber("simu_send_xy", Point, sub_xy)
		rospy.Subscriber("simu_send_wind", Point, sub_wind)
		pub_send_uq = rospy.Publisher('control_send_uq', Point, queue_size=10)
		uq_msg = Point()
		while not rospy.is_shutdown():
			x = array([[pos_x,pos_y,theta]]).T
			u, q  = controle(x,q)

			clear(ax)
			draw_sailboat(x,delta_s,u[0,0],psi,awind)
			ax.plot([a[0,0],b[0,0]], [a[1,0],b[1,0]])

			uq_msg.x = u[0,0]
			uq_msg.y = u[1,0]
			uq_msg.z = q
			pub_send_uq.publish(uq_msg)
			#rospy.loginfo(delta_s)
	except rospy.ROSInterruptException:
		pass