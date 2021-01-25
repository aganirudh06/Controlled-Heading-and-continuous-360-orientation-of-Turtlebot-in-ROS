#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#Defining desired heading (Degrees)
goal_angle = np.deg2rad(0)

#Define desired velocity(m/s)
desired_velocity = 0.5

#Initialising Global Variables
switch = 0

#Gains for controllers designed for rotation and forward movement
x_gain = 0.9
y_gain = 2.2

# Publisher for sending velocity commands to Turtlebot
pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)


def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('polaris', anonymous=True)
     
    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/odom", Odometry,rotate)
    
    # Ros spin to prevent program from exiting
    rospy.spin()


def turtlemove():
    #Defining scope of the variables as global
    global switch
    global goal_angle
    global goal_x
    global goal_y
    global x_gain
    global y_gain
    global desired_velocity
    
    #Cloning the Twist method into a variable	
    move = Twist()

    #Assuring no unnecessary movements
    move.linear.y = 0
    move.linear.z = 0
    move.angular.x = 0
    move.angular.y = 0

    #Switch Cases

    #Switch Case 0
    if switch ==0:

        #Calculating and defining the error between current heading and desired heading (radians)
        error_angle = goal_angle-yaw
        
        #P controller for controlled rotation of turtlebot and publishing required rotation to maintain the error at zero  
        move.angular.z = x_gain*(error_angle)    
	move.linear.x = 0        
	pub.publish(move)
        
        #Switching the state after turtlebot's heading is now the desired heading
        if error_angle<0.001:
            move.angular.z = 0
            pub.publish(move)
            switch=1

    #Switch Case 1
    elif switch ==1:

	#Calculating and defining the error between current heading and desired heading (radians) and current (x,y) and desired (x,y) (m,m)
	error_angle = (goal_angle)-(yaw)
	
        #P controller for maintaing the desired heading and moving the turtlebot forward at a desired velocity 
	move.angular.z = y_gain*(error_angle)
	move.linear.x = desired_velocity
	pub.publish(move)


def rotate(msg):
    #Defining yaw as a global variable so that it can be used for turtlebot movement in turtlemove function
    global yaw

    #Reading the turtlebot's orientation from odometry
    orientation_q = msg.pose.pose.orientation
    #Converting the orientation from quaternion to euler or radians
    orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

    #Calling the turtlemove function to move the turtlebot
    turtlemove()



if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
