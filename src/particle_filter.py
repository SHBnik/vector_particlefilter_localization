#! /usr/bin/env python
import numpy as np
import rospy
import random
import time
import math
from numpy.random import uniform
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

x = 0.0
y = 0.0 
theta = 0.0

x = 0.0
y = 0.0 
theta = 0.0

PI = 3.1415926535897
 
 
last_x=0
last_y=0

get_last_loc_flag = True
input_state = True
yaw_setpoint = 0
r_setpoint = 0
r_max_p_term = 0.3

def callback(msg):
    print(msg.range)
    
def new_odometry(msg):
    global x
    global y
    global theta
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("particle_filter")

sub = rospy.Subscriber("/odom", Odometry, new_odometry)
pub = rospy.Publisher('/vector/cmd_vel', Twist, queue_size = 1)
laser = rospy.Subscriber('/vector/laser', Range, callback)
speed = Twist()

landmarks = np.array([ [0.5, 0.5], [0.5, -0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, 0]  ])
number = 1000

particles = np.empty((number, 2))
particles[:, 0] = uniform(-1, 1, size=number)
particles[:, 1] = uniform(-1, 1, size=number)

weights = np.array([1.0]*number)/number

r = rospy.Rate(50) #50 Hz

while not rospy.is_shutdown():
    if input_state :
        time.sleep(2)

        yaw_setpoint = random.randrange(0, 180)
        r_setpoint = random.randrange(0, 100) / 100.0
        if math.sqrt(x**2 + y**2) + r_setpoint >= 1 :
            r_setpoint = 1 - math.sqrt(x**2 + y**2)
        yaw_setpoint = yaw_setpoint * PI / 180
        
        print(r_setpoint, yaw_setpoint)
        input_state = False
        
        # predict particles
        E_rot = np.random.normal(0.9957*yaw_setpoint + 0.0768, 0.0005, number)
        E_trans = np.random.normal(0.1021, 0.0015, number)
        t = E_rot + E_trans
        t = t * PI / 180
        
        E_trans = np.random.normal(0.99*r_setpoint + 0.0034, 0.0027, number)
        E_rot = np.random.normal(0.0125, 0.0007, number)
        dist = E_trans + E_rot

        particles[:, 0] += np.cos(t) * dist
        particles[:, 1] += np.sin(t) * dist
        
    else:
        yaw_error = yaw_setpoint - theta
        speed.angular.z = yaw_error * 10
        if yaw_error < 0.01 and yaw_error > -0.01:
            
            if get_last_loc_flag :
                last_x = x
                last_y = y
                get_last_loc_flag = False
            r_error = r_setpoint - math.sqrt((x-last_x)**2 + (y-last_y)**2)
            r_p_term = r_error * 2
            if r_p_term > r_max_p_term :   
                r_p_term = r_max_p_term
            speed.linear.x = r_p_term
            if r_error < 0.001 and r_error > -0.001:
                speed.linear.x = 0
                get_last_loc_flag = True
                input_state = True
                
                    # sensor update
                
                # resample
                # 5 roulette wheel
                wheel_sum = np.sum(weights)
                pointers = np.random.rand(5) * wheel_sum
                cum_wheel = np.cumsum(weights)
                indexes = np.zeros((number, 1))
                i = 0
                while i < number:
                    for j in range(5):
                        indexes[i] = np.where(pointers[j] <= cum_wheel)[0][0]
                        i += 1
                    pointers +=  np.random.rand(5) * wheel_sum
                    pointers[np.where(pointers > wheel_sum)[0]] -= wheel_sum
                indexes = np.ravel(indexes)
                indexes = indexes.astype(int)
                particles[:] = particles[indexes]
                weights = weights[indexes]
                weights /= np.sum(weights)
                
                print("done!", x, theta)
            
            
 
        pub.publish(speed)


    r.sleep()
