#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from math import atan2

x = 5.0
y = 5.0
i = 0
theta = 0.0
list1 = []
list2 = []
status = 0


def newOdom(msg):
    global x
    global y
    global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def getPos1(data):
    global strData2
    strData2 = data.data
    global list2
    list2.append(strData2)
    rospy.loginfo("Got Location from Robot1")
    rospy.loginfo(strData2)


def getStatus(data):
    global status
    status = int(data.data)
    rospy.loginfo("Got Status:")
    rospy.loginfo(status)


rospy.init_node("RobotController3")

rospy.Subscriber("/robot3/odom", Odometry, newOdom)
pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=1)
pubPos = rospy.Publisher('Robot3', String, queue_size=10)
pubStat = rospy.Publisher('Robot3Status', String, queue_size=10)
rospy.Subscriber("Robot1", String, getPos1)
rospy.Subscriber("controller", String, getStatus)

speed = Twist()
r = rospy.Rate(5)
r.sleep()
pubStat.publish("1")

while status < 3:
    r.sleep()
status = 0
px = str(round(x, 2))
py = str(round(y, 2))
strMsg = px + "@" + py
rospy.loginfo(strMsg)
pubPos.publish(strMsg)
r.sleep()
goal1 = Point()
pubStat.publish("1")
while status < 3:
    r.sleep()
status = 0
rospy.loginfo("Starting Iterations:")
global i1
i1 = 0
goal1 = Point()
while 1:
    s1 = len(list2)
    if i1 <= s1 - 1:
        localstr1 = list2[i1]
        a1, b1 = localstr1.split("@")
        global x1
        global y1
        x1 = float(a1)
        y1 = float(b1)
    else:
        break
    rospy.loginfo("Points of Robot 1")
    rospy.loginfo(x1)
    rospy.loginfo(y1)
    rospy.loginfo("Goal Position:")
    goal1.x = 0.6 * x1 + 0.4 * x
    goal1.y = 0.6 * y1 + 0.4 * y
    rospy.loginfo(goal1.x)
    rospy.loginfo(goal1.y)
    inc_x = goal1.x - x
    inc_y = goal1.y - y
    angle_to_goal = atan2(inc_y, inc_x)
    while abs(goal1.x - x) > 0.2 or abs(goal1.y - y) > 0.2:
        if abs(angle_to_goal - theta) > 0.15:
            speed.linear.x = 0.0
            speed.angular.z = 0.5
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
        pub.publish(speed)
        r.sleep()
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)
    rospy.loginfo("Finished Iteration")
    i1 = i1 + 1
    px = str(round(x, 2))
    py = str(round(y, 2))
    strMsg = px + "@" + py
    rospy.loginfo(len(list1))
    rospy.loginfo(i1)
    pubPos.publish(strMsg)
    rospy.loginfo("Published Position")
    r.sleep()
    pubStat.publish("1")
    rospy.loginfo("Published Status")
    r.sleep()
    rospy.loginfo("Waiting for Status:")
    while status < 3:
        pub.publish(speed)
    if status==4:
        break
        rospy.loginfo("Stoping the movement")
    rospy.loginfo("Got Next iteration points")
    status = 0