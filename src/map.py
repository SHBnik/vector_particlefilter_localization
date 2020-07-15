import xml.etree.ElementTree as ET
# import matplotlib.patches as patches
import matplotlib.pyplot as plt
import math




def init_map(address):

    tree = ET.parse(address)
    root = tree.getroot()


    rects = []

    for links in root[0].iter('model'):
    
        try:

            for link in links: 
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
    
    return rects


def intersection(p1,p2,p3,p4):
    # example : x1 = 2 and x2 = 4
    if (p2[0] - p1[0] == 0) and (p4[0] - p3[0] == 0):
        # example : x1 = 2 and x2 = 2
        if p1[0] == p3[0]:
            return p3
        # example : x1 = 2 and x2 = 3
        # parallel lines
        else:
            return False
    elif (p2[0] - p1[0] == 0):
        x = p1[0]
        m2 = (p4[1] - p3[1])/float(p4[0] - p3[0])
        y = m2*(x - p3[0]) + p3[1]
    elif (p4[0] - p3[0] == 0):
        x = p3[0]
        m1 = (p2[1] - p1[1])/float(p2[0] - p1[0])
        y = m1*(x - p1[0]) + p1[1]
    else:
        m1 = (p2[1] - p1[1])/float(p2[0] - p1[0])
        m2 = (p4[1] - p3[1])/float(p4[0] - p3[0])

        # parallel lines
        if m1 == m2 :
            return False
        x = (p3[1] - p1[1] + m1*p1[0] - m2*p3[0]) / (m1 - m2)
        y = m1*(x - p1[0]) + p1[1]
    
    epsilon = 0.00001
    if x >= min(p1[0],p2[0])-epsilon and x <= max(p1[0],p2[0])+epsilon and
       y >= min(p1[1],p2[1])-epsilon and y <= max(p1[1],p2[1])+epsilon:
        return x, y

    return False

def convert_point_to_line(rects):
    lines = []
    for points in rects:
        lines.append([ points[0] , points[1]] )
        lines.append([ points[1] , points[3]] )
        lines.append([ points[3] , points[2]] )
        lines.append([ points[2] , points[0]] )
    return lines

def plot_map(rects):
    for rect in rects:
        rect = list(zip(*rect))
        plt.plot(rect[0], rect[1], c='black')

