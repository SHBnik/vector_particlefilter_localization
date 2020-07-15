import xml.etree.ElementTree as ET
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
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


# lims = (-1, 1)


# plt.ylim(lims)
# plt.xlim(lims)




# plt.show()



def _line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection(p1,p2,p3,p4):

    L1 = _line(p1, p2)
    L2 = _line(p3, p4)

    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D

        epsilon = 0.00001

        if x >= min(p1[0],p2[0])-epsilon and x <= max(p1[0],p2[0])+epsilon \
            and x >= min(p3[0],p4[0])-epsilon and x <= max(p3[0],p4[0])+epsilon:
            if y >= min(p1[1],p2[1])-epsilon and y <= max(p1[1],p2[1])+epsilon \
                and y >= min(p3[1],p4[1])-epsilon and y <= max(p3[1],p4[1])+epsilon:
                return x,y

        return False
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
