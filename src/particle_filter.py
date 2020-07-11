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



#+++++++++++++++++++++ init vars ++++++++++++++++++++

x = 0.0
y = 0.0 
theta = 0.0
 
 
last_x=0
last_y=0

get_last_loc_flag = True
random_move_state = True
yaw_setpoint = 0
r_setpoint = 0
r_max_p_term = 0.3

map_address = ''

laser_data = 0
new_laser_data_flag = False

distance_threshold = 0.26

particle_number = 5000

x_limit = 1
y_limit = 1
#------------------------------------------------------



#+++++++++++++++++++++ functions +++++++++++++++++++++++

def calculate_normal_pdf(d,z,variance):
    #   we want to calculate p(z|d)
    #   d is numpy array
    p = np.array([])
    for dist in d:
        #   by dividing the pdf by it max we gonna map it between 0 to 1
        p = np.append(p, math.exp((-0.5) * (((z-dist)/variance)**2.0)))
    
    return p

#-------------------------------------------------------




#+++++++++++ read vector laser ranger finder +++++++++++

def laser_reader(msg):
    global new_laser_data_flag,laser_data

    new_laser_data_flag = True
    laser_data = msg.range

#------------------------------------------------------
 


#++++++++++++ read new pose from the gazebo world +++++++

def new_odometry(msg):
    global x
    global y
    global theta
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

#------------------------------------------------------



#++++++++++++++++++ ROS init ++++++++++++++++++++++++++

rospy.init_node("particle_filter")

sub = rospy.Subscriber("/odom", Odometry, new_odometry)
pub = rospy.Publisher('/vector/cmd_vel', Twist, queue_size = 1)
laser = rospy.Subscriber('/vector/laser', Range, laser_reader, queue_size = 1)
speed = Twist()

#------------------------------------------------------



#+++++++++++++++++++++ MAP ++++++++++++++++++++++++++++++
'''
TODO: add map_reader code to read the world file 
'''
landmarks = np.array([ [0.5, 0.5], [0.5, -0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, 0]  ])

#--------------------------------------------------------

#+++++++++++++++++ init particles +++++++++++++++++++++++
particles = np.empty((particle_number, 3))
particles[:, 0] = uniform(-1, 1, size=particle_number)
particles[:, 1] = uniform(-1, 1, size=particle_number)
particles[:, 2] = random.choice([-90,90,180,0], size=particle_number)

weights = np.array([1.0]*particle_number)/particle_number
#---------------------------------------------------------

while not rospy.is_shutdown():
    
    if random_move_state :

        time.sleep(2)
        
        r_setpoint = max(x_limit, y_limit)
        yaw_setpoint = 0
        #   choose a random move for vector 
        #   which is acceptable for map
        '''
            TODO: Nastaran, add landmark collision.
        '''
        while (abs(x + r_setpoint*math.cos(yaw_setpoint)) >= x_limit \
               or abs(y + r_setpoint*math.sin(yaw_setpoint)) >= y_limit):
            yaw_setpoint = random.choice([-90,90,180,0])
            r_setpoint = random.choice([0.2,0.3,0.5])

        yaw_setpoint = yaw_setpoint * math.pi / 180
        
        #------------------------------------------------------------
        #++++++++++++++++++++ predict (move) particles ++++++++++++++
        e_rot_in_rot = np.random.normal(0.9957*yaw_setpoint + 0.0768, 0.0005, particle_number)
        e_trans_in_rot = np.random.normal(0.1021, 0.0015, particle_number)
        total_rotation = e_rot_in_rot + e_trans_in_rot

        total_transition = 0
        e_trans_in_trans = np.random.normal(0.99*r_setpoint + 0.0034, 0.0027, particle_number)
        e_rot_in_trans = np.random.normal(0.0125, 0.0007, particle_number)
        total_transition = e_trans_in_trans + e_rot_in_trans

        particles[:, 0] += np.cos(total_rotation) * total_transition
        particles[:, 1] += np.sin(total_rotation) * total_transition
        particles[:, 2] = total_rotation
        #------------------------------------------------------------
        
        print(r_setpoint, yaw_setpoint)
        random_move_state = False

    else:
        #++++++++++++++++++ move robot in gazebo ++++++++++++++++
        #   calculte the yaw error
        yaw_error = yaw_setpoint - theta
        # move yaw
        speed.angular.z = yaw_error * 10
        if yaw_error < 0.01 and yaw_error > -0.01:
            
            #   stop the rotation
            speed.angular.z = 0
            pub.publish(speed)

            # wait for new laser data
            new_laser_data_flag = False 
            while not new_laser_data_flag:pass
            last_laser_data = laser_data
            
            """
            I ignore wall collision in choosing random move part with while command
            So I think this if condition is not necessary.
            """
            # validate the distance to wall if less than the threshold dont apply the transistion
            if last_laser_data > distance_threshold:
                # save teh first place location 
                if get_last_loc_flag :
                    last_x = x
                    last_y = y
                    get_last_loc_flag = False
                # calculate the transition error 
                r_error = r_setpoint - math.sqrt((x-last_x)**2 + (y-last_y)**2)
                r_p_term = r_error * 2
                # limit the transition speed 
                if r_p_term > r_max_p_term :   
                    r_p_term = r_max_p_term
                # move linear
                speed.linear.x = r_p_term
                if r_error < 0.001 and r_error > -0.001:
                    speed.linear.x = 0
                    get_last_loc_flag = True
                    # prepare for move randomly again 
                    random_move_state = True
            
                    #++++++++++++++++++++ sensor update ++++++++++++++++++++++++++

                    #   calculate the distance of particles and wall in the map
                    '''
                    TODO: shahab find the formula im tired now for that :)
                    '''
                    particles_distance = 0
                    #   sensor model is a normal distribution [mean,var]
                    #   mean is the distance that particles read 
                    #   var is 0.000097
                    weights = calculate_normal_pdf(particles_distance , laser_data , 0.000097)
                    #-------------------------------------------------------------    

                    #+++++++++++++++++++++++ resample +++++++++++++++++++++++++++++
                    # 5 pointer stochastic universal sampling
                    weights /= np.sum(weights)
                    # 5 random position in wheel for initial pointer position
                    pointers = np.random.rand(5)
                    cum_wheel = np.cumsum(weights)
                    indexes = np.zeros((particle_number, 1))
                    i = 0
                    while i < particle_number:
                        # find index of particles which 5 pointers pointed
                        for j in range(5):
                            indexes[i] = np.where(pointers[j] <= cum_wheel)[0][0]
                            i += 1
                        # roll wheel
                        pointers +=  np.random.rand(5)
                        # move exceeded pointers to acceptable place
                        pointers[np.where(pointers > 1)[0]] -= 1
                        
                    indexes = np.ravel(indexes)
                    indexes = indexes.astype(int)
                    # update particles and weights
                    particles[:] = particles[indexes]
                    weights = weights[indexes]
                    #--------------------------------------------------------------
            
            
 
        pub.publish(speed)


