#!/usr/bin/env python2
from numpy import *
import numpy as np

def control(theta,thetabar,psi,gain):
	u_rudder = gain*np.arctan(np.tan(0.5*(sawtooth(theta-thetabar))))/2
	u_rudder = min(u_rudder,np.pi/4)
	u_rudder = max(u_rudder,-np.pi/4)
	u_sail = np.pi/4*(np.cos(psi-thetabar)+1)
	return u_rudder, u_sail

##############################################################################################
#      Cap following
##############################################################################################

def control_cap_following(theta,thetabar,psi,gain):
	u_rudder = gain*np.arctan(np.tan(0.5*(sawtooth(theta-thetabar))))/2
	u_rudder = min(u_rudder,np.pi/4)
	u_rudder = max(u_rudder,-np.pi/4)
	u_sail = np.pi/4*(np.cos(psi-thetabar)+1)
	return u_rudder, u_sail

##############################################################################################
#      Line following
##############################################################################################

def get_thetabar_line_following(x,q,a,b,psi):
	r = 5
	zeta = pi/4

	x=x.flatten()
	theta=x[2]
	m = array([[x[0]], [x[1]]])
	e = linalg.det(hstack(((b-a)/linalg.norm(b-a), m-a)))
	phi = arctan2(b[1,0]-a[1,0], b[0,0]-a[0,0])

	if abs(e) > 10:
	    q = sign(e)

	thetabar = phi - arctan(e/r)
	if (cos(psi-theta)+cos(zeta))<0: #(cos(psi-thetabar)+cos(zeta))<0:
		#rospy.loginfo("[controler] psi:{},theta:{},thetabar:{},q:{}".format(psi,theta,thetabar,q))
		thetabar = pi+psi-zeta*q

	return thetabar

def control_line_following(x,thetabar,psi):

	theta = x[2,0]
	deltar = 2/pi*arctan(tan(0.5*(theta-thetabar)))
	deltamax = pi/4*(cos(psi-thetabar)+1)
	#rospy.loginfo((sawtooth(psi-theta)/2,pi/4*(cos(psi-thetabar)+1)))
	u = array([[deltar],[deltamax]])
	return u

##############################################################################################
#      Station keeping
##############################################################################################

def get_thetabar_station_keeping(control_params,x,zone):
	"""
	params :
		x_origin : x coordiante of the origin
		y_origin : y coordiante of the origin
		theta_wind : orientation of the wind
		l : distance between origin and center of circles
		r : radius of circles
	x :
		x : x coordiante of the boat
		y : y coordiante of the boat
		theta : heading of the boat
	"""
	x_origin,y_origin,theta_wind,l,r = control_params

	x_or = x_origin + np.cos(theta_wind+np.pi/2)*l
	y_or = y_origin + np.sin(theta_wind+np.pi/2)*l
	x_ol = x_origin + np.cos(theta_wind-np.pi/2)*l
	y_ol = y_origin + np.sin(theta_wind-np.pi/2)*l

	x_ur = x_or - np.cos(theta_wind)*r
	y_ur = y_or - np.sin(theta_wind)*r
	x_dr = x_or + np.cos(theta_wind)*r
	y_dr = y_or + np.sin(theta_wind)*r

	x_ul = x_ol - np.cos(theta_wind)*r
	y_ul = y_ol - np.sin(theta_wind)*r
	x_dl = x_ol + np.cos(theta_wind)*r
	y_dl = y_ol + np.sin(theta_wind)*r

	#print(zone)
	if zone == 1:
		thetabar = line_following(x_dl,y_dl,x_ur,y_ur,x)
		out_of_zone = np.linalg.det(np.array([[x_ur-x[0,0], x_ur-x_dr],[y_ur-x[1,0],y_ur-y_dr]]))
		if out_of_zone<0:
			zone = 2
			print("[controler] Zone 2")
	elif zone == 2:
		thetabar = circle_following(x_or,y_or,r,-1,x)
		out_of_zone = np.linalg.det(np.array([[x_dr-x[0,0], x_dr-x_ur],[y_dr-x[1,0],y_dr-y_ur]]))
		if out_of_zone<0:
			zone = 3
			print("[controler] Zone 3")
	elif zone == 3:
		thetabar = line_following(x_dr,y_dr,x_ul,y_ul,x)
		out_of_zone = -np.linalg.det(np.array([[x_ul-x[0,0], x_ul-x_dl],[y_ul-x[1,0],y_ul-y_dl]]))
		if out_of_zone<0:
			zone = 4
			print("[controler] Zone 4")
	elif zone == 4:
		thetabar = circle_following(x_ol,y_ol,r,1,x)
		out_of_zone = -np.linalg.det(np.array([[x_dl-x[0,0], x_dl-x_ul],[y_dl-x[1,0],y_dl-y_ul]]))
		if out_of_zone<0:
			zone = 1
			print("[controler] Zone 1")
	else: # none existant zone
		thetabar = Nan

	return thetabar,zone,[x_or,y_or,x_ol,y_ol,x_ur,y_ur,x_dr,y_dr,x_ul,y_ul,x_dl,y_dl]

def line_following(x_start,y_start,x_end,y_end,x):
	x = x.flatten()
	x1, x2, theta = x[0], x[1], x[2]

	ratio = 1
	a,b = np.array([[x_start],[y_start]]), np.array([[x_end],[y_end]])
	m = np.array([[x1],[x2]])
	e = np.linalg.det(np.hstack(((b-a)/np.linalg.norm(b-a), m-a)))
	#print(e)
	phi = np.arctan2(b[1,0]-a[1,0], b[0,0]-a[0,0])
	thetabar = phi - np.arctan(e/ratio)

	return thetabar

def circle_following(x_center,y_center,radius,rotation_direction,x):
	x = x.flatten()
	x1, x2, theta = x[0], x[1], x[2]

	x1, x2 = rotation_direction*(x1-x_center)/radius, (x2-y_center)/radius
	#print(x1,x2,(x1**2+x2**2)**0.5)
	VX    = rotation_direction*(-x1**3-x1*(x2**2)+x1-x2)
	VY    = -x2**3-x2*(x1**2)+x1+x2
	R = np.sqrt(VX**2+VY**2)

	thetabar = np.arctan2(VY,VX)

	#print(VX,VY,thetabar)

	return thetabar

def sawtooth(x):
	return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

def control_station_keeping(thetabar,x,psi):
	x = x.flatten()
	x1, x2, theta = x[0], x[1], x[2]

	u = np.arctan(np.tan(0.5*(sawtooth(theta-thetabar)))) *1.5
	u = max(u,-np.pi/4)
	u = min(u,np.pi/4)
	print(thetabar, theta)
	deltamax = pi/4*(cos(psi-theta)+1)
	u = np.array([[u],[deltamax]])
	return u