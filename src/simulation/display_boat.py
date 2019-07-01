#!/usr/bin/env python2
from numpy import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import cos,sin

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

##############################################################################################
#      ROS subscribers
##############################################################################################

def sub_u_rudder(data): # Float32
    global u_rudder
    u_rudder = data.data
    #rospy.loginfo("u_rudder : %s", u_rudder)

def sub_u_sail(data): # Float32
    global u_sail
    u_sail = data.data
    #rospy.loginfo("u_sail : %s", u_sail)

def sub_wind_force(data): # Float32
    global wind_force
    wind_force = data.data
    #rospy.loginfo("wind_force : %s", wind_force)

def sub_wind_direction(data): # Float32
    global wind_direction
    wind_direction = data.data
    #rospy.loginfo("wind_direction : %s", wind_direction)

def sub_pos(data): # Pose2D
    global x,y,theta
    x = data.x
    y = data.y
    theta = data.theta
    #rospy.loginfo("x : %s, y : %s, theta : %s", x,y,theta)

def sub_euler_angles(data): # Vector3
    global theta
    theta = -data.x
    #rospy.loginfo("theta : %s",theta*180/np.pi)

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
    R1 = np.matmul(R, hull)
    R2 = np.matmul(np.matmul(R, Rs), sail)
    R3 = np.matmul(np.matmul(R, Rr), rudder)
    plot2D(R1,'black')
    plot2D(R2,'red',2)
    plot2D(R3,'red',2)
    draw_arrow(x[0]+5,x[1],psi,5*awind,'blue')

def plot2D(M,col='black',w=1):
    plt.plot(M[0, :], M[1, :], col, linewidth = w)

def draw_arrow(x,y,theta,L,col):
    e=0.2
    M1=L*np.array([[0,1,1-e,1,1-e],[0,0,-e,0,e]])
    M=np.append(M1,[[1,1,1,1,1]],axis=0)
    R=np.array([[cos(theta),-sin(theta),x],[sin(theta),cos(theta),y],[0,0,1]])
    plot2D(np.matmul(R, M),col)

##############################################################################################
#      Main
##############################################################################################

if __name__ == '__main__':

    rospy.init_node('display_boat')

    u_rudder,u_sail,wind_force,wind_direction = 0.7,0.5,1,0
    x,y,theta = 0,0,0

    rospy.Subscriber("control_send_u_rudder", Float32, sub_u_rudder)
    rospy.Subscriber("control_send_u_sail", Float32, sub_u_sail)
    rospy.Subscriber("XXX_send_wind_force", Float32, sub_wind_force)
    rospy.Subscriber("filter_send_wind_direction", Float32, sub_wind_direction)
    rospy.Subscriber("XXX_send_pos", Pose2D, sub_pos)
    rospy.Subscriber("filter_send_euler_angles", Vector3, sub_euler_angles)

    ax=init_figure(-20,20,-20,20)

    while not rospy.is_shutdown():
        X = array([[x,y,theta]]).T

        clear(ax)
        draw_sailboat(X,u_sail*np.sign(-(wind_direction-theta)),-u_rudder,wind_direction,wind_force)
