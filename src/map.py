import xml.etree.ElementTree as ET
# import matplotlib.patches as patches
import matplotlib.pyplot as plt
import math
import shapely
from shapely.geometry import LineString,Point, Polygon






def init_map(address):

    tree = ET.parse(address)
    root = tree.getroot()


    rects = []

    for links in root[0].iter('model'):
    
        try:

            for link in links: 
                if link.tag == 'pose':
                    global_map_pose = link.text.split(' ')
                if link.tag == 'link':

                    geometry = None 
                    pose = None
                    for _pose in link.iter('pose'):
                        pose = _pose.text.split(' ')
                        break
                    for collision in link.iter('collision'):
                        geometry = collision[3][0][0].text.split(' ')


                    p1 = [float(pose[0]) + (float(geometry[0]) * math.cos(float(pose[5])) / 2)
                            + float(geometry[1]) * math.sin(float(pose[5])) / 2 ,
                            float(pose[1]) + float(geometry[0]) * math.sin(float(pose[5])) / 2
                                - float(geometry[1]) * math.cos(float(pose[5])) / 2 ]

                    p2 = [float(pose[0]) + float(geometry[0]) * math.cos(float(pose[5])) / 2
                        -float(geometry[1])*math.sin(float(pose[5]))/2 ,
                            float(pose[1]) + float(geometry[0])*math.sin(float(pose[5]))/2
                                +float(geometry[1])*math.cos(float(pose[5]))/2]

                    p3 = [float(pose[0]) - float(geometry[0])*math.cos(float(pose[5]))/2
                        +float(geometry[1])*math.sin(float(pose[5]))/2 ,
                            float(pose[1]) - float(geometry[0])*math.sin(float(pose[5]))/2
                                -float(geometry[1])*math.cos(float(pose[5]))/2]

                    p4 = [float(pose[0]) - float(geometry[0])*math.cos(float(pose[5]))/2
                        -float(geometry[1])*math.sin(float(pose[5]))/2 ,
                            float(pose[1]) - float(geometry[0])*math.sin(float(pose[5]))/2
                                +float(geometry[1])*math.cos(float(pose[5]))/2]


                    rects.append([p1 ,p2 ,p3 ,p4])
                
                    # plt.plot([p1[0],p2[0],p4[0],p3[0],p1[0]],[p1[1],p2[1],p4[1],p3[1],p1[1]])
                    
        except Exception as e : pass
    

    # plt.show()
    return rects,global_map_pose


def find_intersection(p1,p2,p3,p4):

    line1 = LineString([tuple(p1), tuple(p2)])
    line2 = LineString([tuple(p3), tuple(p4)])

    int_pt = line1.intersection(line2)
    if int_pt:
        point_of_intersection = int_pt.x, int_pt.y
        return point_of_intersection
    else:
        return False


def convert_point_to_line(rects):
    lines = []
    for points in rects:
        lines.append([ points[0] , points[1]] )
        lines.append([ points[1] , points[3]] )
        lines.append([ points[3] , points[2]] )
        lines.append([ points[2] , points[0]] )
    return lines


def add_offset(rects,offset):
    new_rects = []
    for points in rects: 
        new_rects.append(
            [  
                [points[0][0] + offset[0] , points[0][1] + offset[1]  ] ,
                [points[1][0] + offset[0] , points[1][1] + offset[1]  ] ,
                [points[2][0] + offset[0] , points[2][1] + offset[1]  ] ,
                [points[3][0] + offset[0] , points[3][1] + offset[1]  ] 
            ]
        )
    return new_rects


def convert_to_poly(rects):
    polygons = []
    for points in rects:
        polygons.append(Polygon(
            [tuple(points[0]) ,
            tuple(points[1]),
            tuple(points[3]),
            tuple(points[2])
            ]))
    return polygons

def check_is_collition(point , rects):
    p = Point(tuple(point))
    for rect in rects:
        if rect.contains(p):
            return True
    return False

def out_of_range(particle,offset):
    if particle[0] - offset[0] > 0.5 or particle[0] - offset[0] < -0.5:
        return True
    elif particle[1] - offset[1] > 0.5 or particle[1] - offset[1] < -0.5:
        return True
    else:
        return False

def plot_map(rects):
    for rect in rects:
        rect = list(zip(*rect))
        plt.plot(rect[1], rect[0], c='black')






