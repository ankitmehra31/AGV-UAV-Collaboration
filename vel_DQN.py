#!/usr/bin/env python3
import rospy
import numpy as np
import torch
import torch.nn as nn
from geometry_msgs.msg import Twist
from custom_msg_python.msg import custom
import math

global lpf_k, prev_vel_x, prev_vel_theta, switch
switch =0
lpf_k=0.2
prev_vel_x=0
prev_vel_theta=0

# Define the DQN network
class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 256)
        self.fc3 = nn.Linear(256, 128)
        self.fc4 = nn.Linear(128, output_dim)
    
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        return self.fc4(x)

# ROS Node for DQN control
class DQNControlNode:
    def __init__(self):
        rospy.init_node('dqn_control_node')
        
        # Initialize DQN
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        state_dim = 5  # [state[0], state[1], state[2], target[0], target[1]]
        self.discrete_actions = self.create_discrete_actions()
        action_dim = len(self.discrete_actions)
        self.dqn = DQN(state_dim, action_dim).to(self.device)
        
        # Load pre-trained model
        model_path = rospy.get_param('~model_path', '/home/chitti/rambo2/src/custom_msg_python/src/dqn_model_sudhar.pth')
        self.load_model(model_path)
        
        # ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/motor_cmd', custom, queue_size=10)
        self.custom_sub = rospy.Subscriber('custom_message', custom, self.custom_callback)
        self.switch_sub = rospy.Subscriber('/odometry', custom, self.callback3)
        self.pubx=rospy.Publisher("/plot_error_DQN",custom,queue_size=10)
        # self.puby=rospy.Publisher("/plot_vel_DQN",custom,queue_size=10)
        
        self.rate = rospy.Rate(10)  # 10 Hz

        # Camera parameters
        self.image_center_x = 0  # Assuming image width is 320
        self.image_center_y = 0  # Assuming image height is 320
        
        # Current state
        self.current_state = np.zeros(5)  # [state[0], state[1], state[2], target[0], target[1]]
    
    def create_discrete_actions(self):
        linear_velocities = [-1.0,-0.75, -0.5, -0.25, 0.0, 0.25, 0.5,0.75, 1.0]
        angular_velocities = [-1.0,-0.75, -0.5, -0.25, 0.0, 0.25, 0.5,0.75, 1.0]
        return [(lv, av) for lv in linear_velocities for av in angular_velocities]
    
    def load_model(self, model_path):
        self.dqn.load_state_dict(torch.load(model_path, map_location=self.device))
        self.dqn.eval()  # Set the model to evaluation mode
        rospy.loginfo("Loaded pre-trained DQN model")
    
    def callback3(self,B):
        global switch
        switch=B.d

    def custom_callback(self, msg):
        global prev_vel_theta, prev_vel_x,lpf_k
        threshold=20
        speed_cap=1
        speed_cap2=1
        # Calculate error between bounding box center and image center
        error_x = msg.coordinates[0] - 160
        error_y = msg.coordinates[1] - 160
        
        # Calculate angle between target and center of the screen
        # T = self.calculate_angle(error_x, error_y)
        angle=math.atan2(error_y,error_x)
        # T=T*180/math.pi
        # angle=270-T
        if error_x ==0 and error_y==0:
            angle=0
        # if T<=-90:
        #     angle=angle-360

        # Update current state
        self.current_state = np.array([
            self.image_center_x, self.image_center_y, angle,
            error_x, error_y
        ])
        
        # self.current_state = np.array([
        #     msg.state_x, msg.state_y, msg.state_orientation,
        #     msg.target_x, msg.target_y
        # ])
        
        # Select action using DQN
        state_tensor = torch.FloatTensor(self.current_state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.dqn(state_tensor)
            action_index = q_values.max(1)[1].item()
        action = self.discrete_actions[action_index]
        
        V_x=action[0]
        V_theta=action[1]
        
        #Putting a cap on linear and angular velocities
        if error_x>=-threshold and error_x<=threshold and error_y>=-threshold and error_y<=threshold:
            V_x=0
            V_theta=0
        
        V_x=V_x*2
        V_theta=V_theta*10
        V_x = np.clip(V_x, -speed_cap, speed_cap)
        V_theta = np.clip(V_theta, -speed_cap2, speed_cap2)

        #Low pass filter applied on velocities
        V_x=lpf_k*V_x+(1-lpf_k)*prev_vel_x
        prev_vel_x=V_x

        prev_vel_theta=lpf_k*V_theta+(1-lpf_k)*prev_vel_theta
        # prev_vel_theta=V_theta
        print("x ",error_x," y ",error_y)
        print("Velocity x ", prev_vel_theta," Velocity theta ",prev_vel_theta)

        # V_theta=V_theta*2
        
        #Publish errors and velocities for the plot
        plot_errors=custom()
        plot_errors.header.stamp = rospy.Time.now()
        plot_errors.dir_l=switch
        plot_errors.pwm_l=V_x
        plot_errors.pwm_r=prev_vel_theta
        plot_errors.d=error_x
        plot_errors.e=error_y
        plot_errors.f=angle
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
        cmd_vel_msg.e = prev_vel_theta  # Rotation
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        # rospy.loginfo(f"State: {self.current_state}, Action: {action}")
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = DQNControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass