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

 
rospy.init_node("controller")
r = rospy.Rate(5)
rospy.Subscriber("Robot1Status", String, Status1)
rospy.Subscriber("Robot2Status", String, Status2)
rospy.Subscriber("Robot3Status", String, Status3)
pubPos = rospy.Publisher('controller', String, queue_size=10)

while 1:
    if i==3:
        pubPos.publish(str(i))
        r.sleep
        i=0
        rospy.loginfo("Publshed the value and reset the status poition:")
		
	
	