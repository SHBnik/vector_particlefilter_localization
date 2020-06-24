#! /usr/bin/env python
 
import rospy
from sensor_msgs.msg import Range
import sys

i = 0


def callback(msg):
    global i
    print(msg.range)
    f.write(str(msg.range)+'\n')
    i += 1
    if i >= 1000:
        f.close()
        print("Done")
        sys.exit
 
rospy.init_node('laser_data_capture')
sub = rospy.Subscriber('/vector/laser', Range, callback)
f = open("laser_data.csv", "a")
rospy.spin()
print("rosdown")
f.close()