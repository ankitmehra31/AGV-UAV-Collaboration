#!/usr/bin/python

import rospy

from custom_msg_python.msg import custom
from tf.transformations import euler_from_quaternion
import math
import serial
import numpy as np


global error_y
error_y=0
global error_x
error_x=0
global error_theta
global theta
e_lx = [0]*10
e_theta = [0]*10
global error_hx
global error_hy
global error_htheta
global flag,switch
switch=0
flag=0
screen_x=320/2
screen_y=320/2
xpid_gains=[0.03,0.0,0.0]               
thetapid_gains=[0.05,0.0,0.0]

erry=[0]
rho_d2=[0]
husk_yaw=[0,0,0,0]
drone_si=[0,0,0,0]

def pidx(error,pid_gain):
    # print("x error",error)
    e_lx.append(error)
    if  len(e_lx) > 10:
        del e_lx[0]
    error_int = float((1/3)*((e_lx[-10]+e_lx[-1])+(4*(e_lx[-2]+e_lx[-4]+e_lx[-6]+e_lx[-8]))+(2*(e_lx[-3]+e_lx[-5]+e_lx[-7]+e_lx[-9]))))            
    p_value=pid_gain[0]*(error)
    i_value=pid_gain[1]*(error_int)
    d_value=pid_gain[2]*((e_lx[-2]-e_lx[-1])/0.1)
    V_x =  p_value + i_value + d_value
    # print("yyyyyy",e_theta[-1],e_theta[-2])
    return V_x , p_value ,i_value, d_value

def pidtheta(error,pid_gain):
    # print("y error",error)
    e_theta.append(error)
    if  len(e_theta) > 10:
        del e_theta[0]
    error_int = float((1/3)*((e_theta[-10]+e_theta[-1])+(4*(e_theta[-2]+e_theta[-4]+e_theta[-6]+e_theta[-8]))+(2*(e_theta[-3]+e_theta[-5]+e_theta[-7]+e_theta[-9]))))           
    p_value=pid_gain[0]*(error)
    i_value=pid_gain[1]*(error_int)
    d_value=pid_gain[2]*((e_theta[-2]-e_theta[-1])/0.1)
    V_theta =  p_value + i_value + d_value
    # print("yyyyyy",e_theta[-1],e_theta[-2])
    return V_theta , p_value ,i_value, d_value

def callback3(B):
    global switch
    switch=B.d

def callback(A):
    global error_hy
    global error_hx
    global error_htheta
    global error_y
    global error_x
    global error_theta
    global X_AGV, Y_AGV, p, q
    global theta
    global flag

    limit_angle=45
    theta_rot=0
    V_theta=0
    e_x=0
    threshold=15
    speed_cap=1
    speed_cap2=1
    T=270    #270 so that theta become zero in initial condition
    #-------------------\\\\\\\\\\\\\\\\------------------------\\\\\\\\\\\\\\\\--------------
    cent_x=A.coordinates[0]
    cent_y=A.coordinates[1]
   # print(cent_y,cent_x)
    error_y=cent_y-screen_y     
    error_hx=-error_y
    error_x=cent_x-screen_x
    error_hy=-error_x
    print("x ",error_x," y ", error_y)

    #Calculation of angle theta 
    T=math.atan2(error_y,error_x)
    T=T*180/math.pi
    theta=270-T
    if error_x ==0 and error_y==0:
        theta=0
    if T<=-90:
        theta=theta-360

#----------------------------------------------------------------------------------------------------------------------------------

    if flag==1:
        theta_rot=theta
        e_x=0
        if theta>-20 and theta<20:
            flag=0
        elif theta>270-limit_angle and theta<270+limit_angle:
            flag=2
            e_x=0
        elif theta>90+limit_angle and theta<270-limit_angle:
            flag=0
            theta_rot=theta-180
            e_x=error_hx

    elif flag==2:
        theta_rot=theta-360
        e_x=0
        if theta>-20 and theta<20:
            flag=0
        elif theta>90-limit_angle and theta<90+limit_angle:
            flag=1
            e_x=0
        elif theta>90+limit_angle and theta<270-limit_angle:
            flag=0
            theta_rot=theta-180
            e_x=error_hx
    else:
        if theta>90-limit_angle and theta<90+limit_angle:
            flag=1
            e_x=0
        elif theta>270-limit_angle and theta<270+limit_angle:
            flag=2
            e_x=0
        elif (theta>0 and theta<limit_angle) or ( theta>270+limit_angle and theta<360):
            if theta<limit_angle and theta>=0 :
                theta_rot=theta
                e_x=error_hx
                print("45 wale me hu")
            elif theta>270+limit_angle:
                theta_rot=theta-360
                e_x=error_hx
                print("315 waale me hu")
        elif theta>90+limit_angle and theta<270-limit_angle:
            flag=0
            theta_rot=theta-180
            e_x=error_hx

    V_x,px,ix,dx=pidx(e_x,xpid_gains)
    V_theta,ptheta,itheta,dtheta=pidtheta(theta_rot,thetapid_gains)

    if error_x>=-threshold and error_x<=threshold and error_y>=-threshold and error_y<=threshold:
        V_x=0
        V_theta=0


    V_x = np.clip(V_x, -speed_cap, speed_cap)
    V_theta = np.clip(V_theta, -speed_cap2, speed_cap2)

    plot_errors=custom()
    plot_errors.header.stamp = rospy.Time.now()
    plot_errors.dir_l=switch
    plot_errors.pwm_l=V_x
    plot_errors.pwm_r=V_theta
    plot_errors.d=error_x
    plot_errors.e=error_y
    plot_errors.f=theta_rot
    pubx.publish(plot_errors)

    print("Vel x", V_x)
    print("vel thetha", V_theta)
    # print("E_x",e_x)
    # print("theta rot",theta_rot)
    # print("flag",flag)
    # print(theta)
    msg = custom()
   # msg.header.frame_id="map"
   # msg.header.stamp = rospy.Time.now()
    msg.d = V_x
    # msg.linear.y = 0.0
    # msg.linear.z = 0.0
    msg.e = V_theta
    pub.publish(msg)




if __name__=='__main__':
    print(1)
    rospy.init_node('transformation', anonymous = True)
    print(2)
    sub2=rospy.Subscriber("custom_message",custom,callback)
    sub3=rospy.Subscriber("/odometry",custom,callback3)
    pub = rospy.Publisher('/motor_cmd', custom, queue_size=1)
    pubx=rospy.Publisher("/plot_error",custom,queue_size=10)

   # rospy.Timer(rospy.Duration(0.02))
    rospy.spin()
