#!/usr/bin/env python3
import rospy
import numpy as np
import torch
import torch.nn as nn
from geometry_msgs.msg import Twist
from custom_msg_python.msg import custom
import math

global switch 
switch =0
# Define the Actor and Critic networks
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(state_dim, 400)
        self.fc2 = nn.Linear(400, 300)
        self.fc3 = nn.Linear(300, action_dim)
        self.max_action = max_action
        
    def forward(self, state):
        a = torch.relu(self.fc1(state))
        a = torch.relu(self.fc2(a))
        return self.max_action * torch.tanh(self.fc3(a))

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(state_dim + action_dim, 400)
        self.fc2 = nn.Linear(400, 300)
        self.fc3 = nn.Linear(300, 1)
        
    def forward(self, state, action):
        q = torch.cat([state, action], 1)
        q = torch.relu(self.fc1(q))
        q = torch.relu(self.fc2(q))
        return self.fc3(q)

# DDPG agent
class DDPG:
    def __init__(self, state_dim, action_dim, max_action, device):
        self.actor = Actor(state_dim, action_dim, max_action).to(device)
        self.device = device
        self.max_action = max_action
    
    def select_action(self, state):
        state = torch.FloatTensor(state.reshape(1, -1)).to(self.device)
        return self.actor(state).cpu().data.numpy().flatten()

# ROS Node for DDPG control
class DDPGControlNode:
    def __init__(self):
        rospy.init_node('ddpg_control_node')
        
        # Initialize DDPG
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        state_dim = 5  # [current_x, current_y, current_theta, target_x, target_y]
        action_dim = 2  # [linear_velocity, angular_velocity]
        max_action = 1.0
        self.ddpg = DDPG(state_dim, action_dim, max_action, self.device)
        
        # Load pre-trained model
        model_path = rospy.get_param('~model_path', '/home/chitti/rambo2/src/custom_msg_python/src/ddpg_model_min_energy.pth')
        self.load_model(model_path)
        
        # ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/motor_cmd', custom, queue_size=1)
        self.custom_sub = rospy.Subscriber('custom_message', custom, self.custom_callback)
        self.switch_sub = rospy.Subscriber('/odometry', custom, self.callback3)
        self.pubx=rospy.Publisher("/plot_error_DDPG",custom,queue_size=10)
        # self.puby=rospy.Publisher("/plot_vel_DDPG",custom,queue_size=10)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Camera parameters
        self.image_center_x = 160  # Assuming image width is 320
        self.image_center_y = 160  # Assuming image height is 320
        
        # Current state
        self.current_state = np.zeros(5)  # [current_x, current_y, current_theta, target_x, target_y]
    
    def load_model(self, model_path):
        checkpoint = torch.load(model_path, map_location=self.device)
        self.ddpg.actor.load_state_dict(checkpoint['actor'])
        rospy.loginfo("Loaded pre-trained DDPG model")
    
    def callback3(self,B):
        global switch
        switch=B.d

    def custom_callback(self, msg):
        threshold=20
        speed_cap=1
        speed_cap2=1
        # Update target position
        target_x = -msg.coordinates[0] + self.image_center_x
        target_y = msg.coordinates[1] - self.image_center_y
        
        # Calculate current position and orientation
        # This is a simplification. In a real scenario, you'd get this from odometry or other sensors
        current_x = 0  # Assume the robot is always at the center of its own coordinate system
        current_y = 0
        if (target_x==0 and target_y==0):
            current_theta=0
        else:
            current_theta = math.atan2(target_y, target_x)  # Orientation towards the target
        
        print("error x ",target_x, "error y ", target_y, "theta ", current_theta*180/3.141)
        # Update current state
        self.current_state = np.array([current_x, current_y, current_theta, target_x, target_y])
        
        # Select action using DDPG
        action = self.ddpg.select_action(self.current_state)
        V_x=action[0]
        V_theta=action[1]
        #Putting a cap on linear and angular velocities
        if target_x>=-threshold and target_x<=threshold and target_y>=-threshold and target_y<=threshold:
            V_x=0
            V_theta=0
            
        # V_x=0
        # V_theta=0.2
        V_theta=-V_theta
        V_x = np.clip(V_x, -speed_cap, speed_cap)
        V_theta = np.clip(V_theta, -speed_cap2, speed_cap2)

        #Publish errors and velocities for the plot
        plot_errors=custom()
        plot_errors.header.stamp = rospy.Time.now()
        plot_errors.dir_l=switch
        plot_errors.pwm_l=V_x
        plot_errors.pwm_r=V_theta
        plot_errors.d=target_x
        plot_errors.e=target_y
        plot_errors.f=current_theta
        self.pubx.publish(plot_errors)

        # plot_vel=custom()
        # plot_vel.header.stamp = rospy.Time.now()
        # plot_vel.dir_l=switch
        # plot_vel.d=V_x
        # plot_vel.e=V_theta
        # self.puby.publish(plot_vel)
        
        # Publish action as cmd_vel
        cmd_vel_msg = custom()
        cmd_vel_msg.d = V_x  # Forward/backward movement
        
        cmd_vel_msg.e = V_theta  # Rotation
        print("Velocity x ", V_x," Velocity theta ",V_theta)
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        # rospy.loginfo(f"State: {self.current_state}, Action: {action}")
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = DDPGControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass