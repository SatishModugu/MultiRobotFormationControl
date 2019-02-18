#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from math import atan2
 
x = 0.0
y = 0.0
theta = 0.0
i=0
x1 = 0.0
y1 = 0.0
x2 = 0.0
y2 = 0.0
x3 = 0.0
y3 = 0.0

def Status1(data):
    global i
    strData = int(data.data)
    i=i+strData
    rospy.loginfo(i)
    rospy.loginfo("Got Status from Robot1")


def Status2(data):
    global i
    global strData2
    strData2 = int(data.data)
    i=i+strData2
    rospy.loginfo(i)
    rospy.loginfo("Got Status from Robot2")
    
	
def Status3(data):
    global i
    global strData3
    strData3 = int(data.data)
    i=strData3+i
    rospy.loginfo(i)
    rospy.loginfo("Got Status from Robot3")

def getPos1(data):
    global strData1
    strData1 = data.data
    rospy.loginfo("Got Location from Robot1")
    rospy.loginfo(strData1)
    a1, b1 = strData1.split("@")
    global x1
    global y1
    x1 = float(a1)
    y1 = float(b1)

def getPos2(data):
    global strData2
    strData2 = data.data
    rospy.loginfo("Got Location from Robot2")
    rospy.loginfo(strData2)
    a2, b2 = strData2.split("@")
    global x2
    global y2
    x2 = float(a2)
    y2 = float(b2)

def getPos3(data):
    global strData3
    strData3 = data.data
    rospy.loginfo("Got Location from Robot2")
    rospy.loginfo(strData3)
    a3, b3 = strData2.split("@")
    global x3
    global y3
    x3 = float(a3)
    y3 = float(b3)


rospy.init_node("controller")
r = rospy.Rate(5)
rospy.Subscriber("Robot1Status", String, Status1)
rospy.Subscriber("Robot2Status", String, Status2)
rospy.Subscriber("Robot3Status", String, Status3)
rospy.Subscriber("Robot1", String, getPos1)
rospy.Subscriber("Robot2",String, getPos2)
rospy.Subscriber("Robot3", String, getPos3)
pubPos = rospy.Publisher('controller', String, queue_size=10)

while 1:
    if i==3:
        if abs(y1 - y2) < 0.8 and abs(y2 - y3) < 0.8 and abs(x1 - x2) < 0.8 and abs(x2 - x3) < 0.8:
            pubPos.publish("4")
        else:
            pubPos.publish(str(i))
        r.sleep
        i=0
        rospy.loginfo("Published the value and reset the status position:")
		
	
	