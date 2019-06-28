#!/usr/bin/env python2
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

##############################################################################################
#      ROS
############################################################################################
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import Imu
def send_imu(pub):
    imu_msg = Imu()
    imu_msg.linear_acceleration.x = 0
    imu_msg.linear_acceleration.y = 0
    imu_msg.linear_acceleration.z = 0
    imu_msg.angular_velocity.x = 0
    imu_msg.angular_velocity.y = 0
    imu_msg.angular_velocity.z = 0
    imu_msg.orientation.x = 0
    imu_msg.orientation.y = 0
    imu_msg.orientation.z = 0
    imu_msg.orientation.w = 0
    pub.publish(imu_msg)
##############################################################################################
#      Display
##############################################################################################

def print_triangle(l_points):
    n = len(l_points)
    for i in range(n):
        plot([l_points[i,0,0],l_points[(i+1)%n,0,0]],[l_points[i,1,0],l_points[(i+1)%n,1,0]],'red')
        plot(l_points[i,0,0],l_points[i,1,0], 'g+')

##############################################################################################
#      Control
##############################################################################################

def f(x,u):
    x,u=x.flatten(),u.flatten()
    θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; δsmax=u[1];
    w_ap = array([[awind*cos(ψ-θ) - v],[awind*sin(ψ-θ)]])
    ψ_ap = angle(w_ap)
    a_ap=norm(w_ap)
    sigma = cos(ψ_ap) + cos(δsmax)
    if sigma < 0 :
        δs = pi + ψ_ap
    else :
        δs = -sign(sin(ψ_ap))*δsmax
    fr = p4*v*sin(δr)
    fs = p3*a_ap* sin(δs - ψ_ap)
    dx=v*cos(θ) + p0*awind*cos(ψ)
    dy=v*sin(θ) + p0*awind*sin(ψ)
    dv=(fs*sin(δs)-fr*sin(δr)-p1*v**2)/p8
    dw=(fs*(p5-p6*cos(δs)) - p7*fr*cos(δr) - p2*w*v)/p9
    xdot=array([ [dx],[dy],[w],[dv],[dw]])
    return xdot,δs    

def controle(x, q):
    x=x.flatten()
    θ=x[2]; v=x[3]; w=x[4]

    m = array([[x[0]], [x[1]]])
    e = det(hstack(((b-a)/norm(b-a), m-a)))
    print(e,q)
    phi = arctan2(b[1,0]-a[1,0], b[0,0]-a[0,0])

    if abs(e) > r:
        q = sign(e)
    
    thetabar = phi - arctan(e/r)

    if (cos(ψ-thetabar)+cos(zeta))<0:
        thetabar = pi+ψ-zeta*q
    
    deltar = 2/pi*arctan(tan(0.5*(θ-thetabar)))
    deltamax = pi/4*(cos(ψ-thetabar)+1)
    u = np.array([[deltar],[deltamax]])
    return u, q

##############################################################################################
#      Main
##############################################################################################

# --- ROS ------- #

pub_send_sail = rospy.Publisher('simu_send_imu', Imu, queue_size=10)
rospy.init_node('simu_send_imu')

# --- Display --- #

ax=init_figure(-100,250,-100,60)

# --- Control --- #

p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.1,1,6000,1000,2000,1,1,2,300,10000
x = array([[10,40,0,1,0]]).T   #x=(x,y,θ,v,w)

dt = 0.5#0.1
awind,ψ = 2,-0 #2,-2
a = array([[-75],[40]])   
b = array([[175],[-40]])
r = 5
zeta = pi/4
q = 1

l_points = np.array([[[-75],[40]],[[175],[-40]],[[200],[20]]])

# --- Main ------- #

for i in range(len(l_points)):
    a = l_points[i]
    b = l_points[(i+1)%len(l_points)]
    while np.linalg.norm(b-x[:2,:]) > 2:
        print(np.linalg.norm(b-x[:2,:]))
        #clear(ax)
        print_triangle(l_points)
        u, q = controle(x, q)
        xdot,δs=f(x,u)
        x = x + dt*xdot
        #draw_sailboat(x,δs,u[0,0],ψ,awind)
        send_imu(pub_send_sail)



        
