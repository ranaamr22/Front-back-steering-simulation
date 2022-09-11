#!/bin/env python3
import rospy 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

x=0
y=0
z=0
yaw=0

rospy.init_node('turtlesim_motion_pose',anonymous=True)

def posecallback(pose_message):
    x=pose_message.x
    y=pose_message.y
    yaw=pose_message.theta

def go_to_goal(linear_vel,back_steer):

    global x
    global y, z, yaw
    velocity_message = Twist()
    start_time=time.time()
    
    

    time_motion=2 
    lf=rospy.get_param("/lf")
    lr=rospy.get_param("/lr")
    cmd_vel_topic = "/turtle1/cmd_vel" 
    velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

    while not rospy.is_shutdown():
        beta=back_steer-math.atan((lr/(lf+lr))*math.tan(back_steer))
        linear_speed_x=linear_vel*math.cos(beta+ yaw)
        linear_speed_y=linear_vel*math.sin(beta+yaw)
        angular_speed=(linear_vel/(lf+lr))*math.cos(back_steer-beta)*math.tan(back_steer)

        velocity_message.linear.x=linear_speed_x
        velocity_message.linear.y=linear_speed_y
        velocity_message.angular.z=angular_speed

        velocity_publisher.publish(velocity_message)
        end_time=time.time()
        duration=start_time+1

        
        if end_time>duration:
            break

rospy.Subscriber('/turtle1/pose',Pose,posecallback)



go_to_goal(2,1)


