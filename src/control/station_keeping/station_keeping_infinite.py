import numpy as np
import matplotlib.pyplot as plt

from roblib import *

def get_thetabar(control_params,x,zone):
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
	elif zone == 2:
		thetabar = circle_following(x_or,y_or,r,-1,x)
		out_of_zone = np.linalg.det(np.array([[x_dr-x[0,0], x_dr-x_ur],[y_dr-x[1,0],y_dr-y_ur]]))
		if out_of_zone<0:
			zone = 3
	elif zone == 3:
		thetabar = line_following(x_dr,y_dr,x_ul,y_ul,x)
		out_of_zone = -np.linalg.det(np.array([[x_ul-x[0,0], x_ul-x_dl],[y_ul-x[1,0],y_ul-y_dl]]))
		if out_of_zone<0:
			zone = 4
	elif zone == 4:
		thetabar = circle_following(x_ol,y_ol,r,1,x)
		out_of_zone = -np.linalg.det(np.array([[x_dl-x[0,0], x_dl-x_ul],[y_dl-x[1,0],y_dl-y_ul]]))
		if out_of_zone<0:
			zone = 1
	else: # none existant zone
		thetabar = Nan

	print("zone : ",zone)
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

def f_car(x,u):
	θ=x[2,0]
	return np.array([[np.cos(θ)], [np.sin(θ)],[u]])

def sawtooth(x):
	return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

def control_car(thetabar,x):
	x = x.flatten()
	x1, x2, theta = x[0], x[1], x[2]

	u = -5*2/np.pi*np.arctan(np.tan(0.5*(sawtooth(theta-thetabar))))
	print(thetabar, theta)
	return u

def f_boat(x,u):
	x,u=x.flatten(),u.flatten()
	θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; δsmax=u[1];
	w_ap = np.array([[awind*np.cos(psi-θ) - v],[awind*np.sin(psi-θ)]])
	psi_ap = angle(w_ap)
	a_ap=np.linalg.norm(w_ap)
	sigma = np.cos(psi_ap) + np.cos(δsmax)
	print(sigma)
	if sigma < 0 :
	    δs = pi + psi_ap
	else :
	    δs = -np.sign(np.sin(psi_ap))*δsmax
	fr = p4*v*np.sin(δr)
	fs = p3*a_ap* sin(δs - psi_ap)
	dx=v*np.cos(θ) + p0*awind*np.cos(psi)
	dy=v*np.sin(θ) + p0*awind*np.sin(psi)
	dv=(fs*np.sin(δs)-fr*np.sin(δr)-p1*v**2)/p8
	dw=(fs*(p5-p6*np.cos(δs)) - p7*fr*np.cos(δr) - p2*w*v)/p9
	xdot=np.array([ [dx],[dy],[w],[dv],[dw]])
	return xdot,δs

def control_boat(x):
	x=x.flatten()
	th=x[2]; v=x[3]; w=x[4]

	deltar = 2/np.pi*np.arctan(np.tan(0.5*(th-thetabar)))
	deltamax = pi/4*(cos(psi-thetabar)+1)
	u = np.array([[deltar],[deltamax]])
	return u


psi = 0.1#np.pi/4
dt = 0.2
params = [15,-30,psi,5,3]
zone = 1
x_car = np.array([[0],[0],[0]])
x_boat = np.array([[3,5,0,1,0]]).T
awind = 2 #2,-2
p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.1,0.1,0.5,3,1,0,0,0.5,10,0.1

thetabar, zone, res = get_thetabar(params,x_boat,zone)


for t in np.arange(0,100,dt):

	plt.cla()
	fut_x_boat = x_boat + 0.5*np.array([[np.cos(x_boat[2,0]),np.sin(x_boat[2,0]),0,0,0]]).T
	thetabar, zone, res = get_thetabar(params,fut_x_boat,zone)
	#u = control(thetabar,fut_x)
	#x = x + dt*f(x,u)
	#plt.plot(x[0,0],x[1,0],'c.')
	u = control_boat(x_boat)
	xdot,delta_s=f_boat(x_boat,u)
	x_boat = x_boat + dt*xdot
	draw_sailboat(x_boat,delta_s,u[0,0],psi,awind)
	plt.plot(0,0,'r+')
	plt.plot(res[0],res[1],'g+')
	plt.plot(res[2],res[3],'b+')
	plt.plot(res[4],res[5],'r+')
	plt.plot(res[6],res[7],'y+')
	plt.plot(res[8],res[9],'r+')
	plt.plot(res[10],res[11],'y+')
	plt.quiver(np.cos(psi),np.sin(psi))
	plt.axis('equal')
	plt.pause(0.01)
	print('temps',t)
plt.show()
