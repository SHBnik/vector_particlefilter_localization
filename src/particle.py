import numpy as np 
from numpy.random import uniform
import math
import scipy.stats as stats
import map

class Particles:
    def __init__(self, number,map_rects,map_lines):
        
        
        self.weights = None
        self.map_rects = map_rects
        self.all_map_lines = map_lines
        self.particle_number = number
        self.particles = np.empty((particle_number, 3))


    def generate_random_particle(self):
        self.particles[:, 0] = uniform(-0.5, 0.5, size=particle_number) + global_map_position[0]
        self.particles[:, 1] = uniform(-0.5, 0.5, size=particle_number) + global_map_position[1]
        self.particles[:, 2] = uniform(-180, 180, size=particle_number)*math.pi/180.0


    def motion_update(self,yaw_setpoint,r_setpoint,laser_data):
        e_rot_in_rot = np.random.normal(0.9957*yaw_setpoint + 0.0768, 0.0005, particle_number)
        e_trans_in_rot = np.random.normal(0.1021, 0.0015, particle_number)
        total_rotation = e_rot_in_rot

        total_transition = 0
        if laser_data > r_setpoint + 0.01:
            e_trans_in_trans = np.random.normal(0.99*r_setpoint + 0.0034, 0.0027, particle_number)
            e_rot_in_trans = np.random.normal(0.0125, 0.0007, particle_number)
            total_transition = e_trans_in_trans + e_rot_in_trans
            total_rotation = e_rot_in_rot + e_trans_in_rot

        self.particles[:, 0] += np.cos(total_rotation) * total_transition
        self.particles[:, 1] += np.sin(total_rotation) * total_transition
        self.particles[:, 2] += (total_rotation - self.particles[:, 2])

    def sensor_update(self):
        self.weights = np.zeros(self.particle_number)
        for i in range(self.particle_number):

            #   calculate the start and the end of sensor line (the length is 0.4)
            #   [start_point , end_point]
            sensor_line = [ [self.particles[i][0], self.particles[i][1]] , \
                [ 0.4*math.cos(self.particles[i][2])+self.particles[i][0] , \
                0.4*math.sin(self.particles[i][2])+self.particles[i][1] ] ]
            
            min_distance = 10
            collission_point = False

            for line in self.all_map_lines:
                #   calculate the intersection point
                intersection_point = map.find_intersection(line[0], line[1] , sensor_line[0], sensor_line[1])

                #   check for the existance of intersection point 
                if intersection_point != False:
                    #   calculate the distance of intersection point and particle position
                    distance = (self.particles[i][0]-intersection_point[0])**2 + (self.particles[i][1]-intersection_point[1])**2 
                    if min_distance >= distance:
                        min_distance = distance
                        collission_point = intersection_point

            #   sensor hit situation
            if collission_point != False:
                #   particle sensor line [ start_point , end_point ]
                particle_sensor_line = [ [ self.particles[i][0] , self.particles[i][1] ], collission_point ]
            #   sensor max situation
            else:
                #   the sensor line is the full lenth sensor line (0.4)
                particle_sensor_line = sensor_line
                min_distance = 0.4

            print(min_distance)
            #   sensor model is a normal distribution [mean,var]
            #   mean is the distance that particles read 
            #   var is 0.000097
            # print min_distance, stats.norm(min_distance, 0.0097).pdf(last_laser_data)
            self.weights[i] = stats.norm(min_distance, 0.0097).pdf(laser_data)

        weights /= np.sum(weights)

    def resample(self):
        pass