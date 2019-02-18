#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from math import atan2
 
x = 2.0
y = 3.0
theta = 0.0
i=0
list1=[]
list2=[]
status =0
 
def newOdom(msg):
    global x
    global y
    global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def getPos2(data):
	global strData2
	strData2=data.data
	global list2
	list2.append(strData2)
	rospy.loginfo("Got Location from Robot2")
	rospy.loginfo(strData2)

def getStatus(data):
    global status
    status = int(data.data)
    rospy.loginfo("Got Status:")
    rospy.loginfo(status)

 
rospy.init_node("RobotController1")
 
rospy.Subscriber("/robot1/odom", Odometry, newOdom)
pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size = 1)
pubPos = rospy.Publisher('Robot1', String, queue_size=10)
pubStat = rospy.Publisher('Robot1Status', String, queue_size=10)
rospy.Subscriber("Robot2",String, getPos2)
rospy.Subscriber("controller", String, getStatus)

speed = Twist()
r = rospy.Rate(5)
r.sleep()
pubStat.publish("1")
while status<3:
    r.sleep()
status=0
r.sleep()
px=str(round(x,2))
py=str(round(y,2))
strMsg=px+"@"+py
rospy.loginfo(strMsg)
pubPos.publish(strMsg)
r.sleep()
pubStat.publish("1")
while status<3:
    r.sleep()
status=0
goal1 = Point()
rospy.loginfo("Starting Iterations:")
global i1
i1=0
goal1 = Point()
while 1:
    s1= len(list2)
    localstr1 = list2[i1]
    a1,b1 = localstr1.split("@")
    global x2
    global y2
    x2=float(a1)
    y2=float(b1)
    rospy.loginfo("Points of Robot 2")
    rospy.loginfo(x2)
    rospy.loginfo(y2)
    goal1.x = 0.8*x+0.2*x2
    goal1.y = 0.8*y+0.2*y2
    rospy.loginfo("Goal Position is:")
    rospy.loginfo(goal1.x)
    rospy.loginfo(goal1.y)
    inc_x = goal1.x-x
    inc_y = goal1.y-y
    angle_to_goal = atan2(inc_y,inc_x)
    while abs(goal1.x-x)>0.15 or abs(goal1.y-y) >0.2:
        if abs(angle_to_goal-theta)>0.15:
            speed.linear.x=0.0
            speed.angular.z=0.5
        else:
            speed.linear.x=0.5
            speed.angular.z=0.0
        pub.publish(speed)
        r.sleep()
    speed.linear.x=0.0
    speed.angular.z=0.0
    pub.publish(speed)
    rospy.loginfo("Finished Iteration")
    rospy.loginfo(x)
    rospy.loginfo(y)
    rospy.loginfo(i1)
    i1=i1+1
    rospy.loginfo("Waiting for next iteration value:")
    rospy.loginfo(len(list1))
    rospy.loginfo(i1)
    rospy.loginfo(status)
    px=str(round(x,2))
    py=str(round(y,2))
    strMsg=px+"@"+py
    pubPos.publish(strMsg)
    r.sleep()
    pubStat.publish("1")
    r.sleep()
    r.sleep()
    rospy.loginfo("Published Status")
    rospy.loginfo("Waiting for Status")
    while  status < 3:
        pub.publish(speed)
        r.sleep()
    if status==4:
        break
        rospy.loginfo("Stoping the movement")
    rospy.loginfo("Got Next iteration points")
    rospy.loginfo("Published Position")
    status=0
