#!/usr/bin/python

import rospy
from custom_msg_python.msg import custom
from tf.transformations import euler_from_quaternion
import math
import numpy as np

global switch
switch =0
# Global variables
error_y, error_x, error_theta = 0, 0, 0
theta = 0
error_hx, error_hy, error_htheta = 0, 0, 0
flag = 0
screen_x, screen_y = 320/2, 320/2

# ISMC parameters
cx, cy = -100, -30
dx, dy = 0.04, 0.04
ex, ey = 0.09, 0.09
nx, ny = 0, 0

# Error history
e_lx, e_ly = [0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0]
e_ltheta = [0,0,0,0,0,0,0,0,0,0]

def ismc_control(error, e_l, c, d, e, n):
    e_l.append(error)
    if len(e_l) > 10:
        del e_l[0]
    
    error_int = (1/3)*((e_l[-10]+e_l[-1])+(4*(e_l[-2]+e_l[-4]+e_l[-6]+e_l[-8]))+(2*(e_l[-3]+e_l[-5]+e_l[-7]+e_l[-9])))
    
    S = c*error + d*error_int
    V = -(e/c)*math.tanh(S) - (1/c)*error - n*((e_l[-1]-e_l[-2])/0.1)
    
    return V, S

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
    global flag,switch

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
    print(" x ",error_x," y ", error_y)

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

    # Apply ISMC
    V_x, Sx = ismc_control(e_x, e_lx, cx, dx, ex, nx)
    V_theta, Stheta = ismc_control(theta_rot, e_ltheta, cy, dy, ey, ny)

    if -threshold <= error_x <= threshold and -threshold <= error_y <= threshold:
        V_x = 0
        V_theta = 0
    
    V_x = np.clip(V_x, -speed_cap, speed_cap)
    V_theta = np.clip(V_theta, -speed_cap2, speed_cap2 )

    errors_smc=custom()
    errors_smc.header.stamp = rospy.Time.now()
    errors_smc.dir_l=switch
    errors_smc.pwm_l=V_x
    errors_smc.pwm_r=V_theta
    errors_smc.d=error_x
    errors_smc.e=error_y
    errors_smc.f=theta_rot
    pubx.publish(errors_smc)
    
    print("Vel x", V_x)
    print("vel theta", V_theta)
    # print("E_x", e_x)
    # print("theta rot", theta_rot)
    # print("flag", flag)
    # print("theta", theta)
    # print("Sx", Sx)
    # print("Stheta", Stheta)

    msg = custom()
    msg.d = V_x
    # msg.linear.y = 0.0
    # msg.linear.z = 0.0
    msg.e = V_theta
    pub.publish(msg)

if __name__ == '__main__':
    print(1)
    rospy.init_node('transformation', anonymous=True)
    print(2)
    sub2 = rospy.Subscriber("custom_message", custom, callback)
    sub3=rospy.Subscriber("/odometry",custom,callback3)
    pub = rospy.Publisher('/motor_cmd', custom, queue_size=1)
    pubx=rospy.Publisher("/plot_error_smc",custom,queue_size=10)
    print(3)
    rospy.spin()