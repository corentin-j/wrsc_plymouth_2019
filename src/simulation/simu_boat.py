#!/usr/bin/env python2

import matplotlib.pyplot as plt
import numpy as np
from numpy import cos,sin

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3


##############################################################################################
#      ROS
##############################################################################################

def sub_uq(data):
    global u, q
    #rospy.loginfo("U_deltar : %s, U_deltamax : %s, Q : %s",data.x, data.y, data.z)
    u = np.array([[data.x],[data.y]])
    q = data.z

##############################################################################################
#      Euler integration
##############################################################################################

def angle(x):
    x=x.flatten()
    return np.arctan2(x[1],x[0])

def f(x,u):
    x,u=x.flatten(),u.flatten()
    theta=x[2]; v=x[3]; w=x[4]; delta_r=u[0]; delta_smax=u[1];
    w_ap = np.array([[awind*cos(psi-theta) - v],[awind*sin(psi-theta)]])
    psi_ap = angle(w_ap)
    a_ap=np.linalg.norm(w_ap)
    sigma = cos(psi_ap) + cos(delta_smax)
    if sigma < 0 :
        delta_s = pi + psi_ap
    else :
        delta_s = -np.sign(sin(psi_ap))*delta_smax
    fr = p4*v*sin(delta_r)
    fs = p3*a_ap* sin(delta_s - psi_ap)
    dx=v*cos(theta) + p0*awind*cos(psi)
    dy=v*sin(theta) + p0*awind*sin(psi)
    dv=(fs*sin(delta_s)-fr*sin(delta_r)-p1*v**2)/p8
    dw=(fs*(p5-p6*cos(delta_s)) - p7*fr*cos(delta_r) - p2*w*v)/p9
    xdot=np.array([[dx],[dy],[w],[dv],[dw]])
    return xdot,delta_s

##############################################################################################
#      Main
##############################################################################################

if __name__ == '__main__':
    try:

        # --- Boat variables --- #

        p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.1,1,6000,1000,2000,1,1,2,300,10000
        x = np.array([[0,30,-2*np.pi/3,1,0]]).T #x=(x,y,theta,v,w)
        dt = 0.1
        pi = np.pi
        awind,psi = 2,-2*np.pi/3 #2,-2

        # --- ROS -------------- #

        pub_send_theta = rospy.Publisher('simu_send_theta', Vector3, queue_size=10)
        pub_send_xy = rospy.Publisher('simu_send_xy', Point, queue_size=10)
        pub_send_wind = rospy.Publisher('simu_send_wind', Point, queue_size=10)
        theta_msg = Vector3()
        xy_msg = Point()
        wind_msg = Point()
        rospy.Subscriber("control_send_uq", Point, sub_uq)
        rospy.init_node('simu_boat')
        rate = rospy.Rate(10) # 10hz

        # --- Main ------------- #
        u, q = np.array([[0],[1.0]]), 1
        while not rospy.is_shutdown():

            xdot,delta_s=f(x,u)
            x = x + dt*xdot
            rospy.loginfo(delta_s)

            theta_msg.x = x[2,0] # heading
            xy_msg.x = x[0,0]              # x pos
            xy_msg.y = x[1,0]              # y pos
            xy_msg.z = delta_s
            wind_msg.x = awind             # wind force/speed
            wind_msg.y = psi               # wind direction

            pub_send_theta.publish(theta_msg)
            pub_send_xy.publish(xy_msg)
            pub_send_wind.publish(wind_msg)

            #rospy.loginfo(xy_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
