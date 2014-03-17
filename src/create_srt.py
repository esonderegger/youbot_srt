#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist


roslib.load_manifest('youbot_srt')


createPub = rospy.Publisher('/create/cmd_vel', Twist)


def createTwist(xVel, yVel, angVel):
    twist = Twist()
    twist.linear.x = xVel
    twist.linear.y = yVel
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angVel
    return twist


def moveBase(xDist, yDist, theta, publisher=createPub):
    timeRequired = 0.1
    if abs(xDist / 0.15) > timeRequired:
        timeRequired = abs(xDist / 0.15)
    if abs(yDist / 0.05) > timeRequired:
        timeRequired = abs(yDist / 0.05)
    if abs(theta / 0.4) > timeRequired:
        timeRequired = abs(theta / 0.4)
    stepsRequired = int(timeRequired / 0.05)
    timeAllocated = stepsRequired * 0.05
    xVel = xDist / timeAllocated
    yVel = yDist / timeAllocated
    tVel = theta / timeAllocated
    publisher.publish(createTwist(0.0, 0.0, 0.0))
    rospy.sleep(0.3)
    for i in range(stepsRequired):
        publisher.publish(createTwist(xVel, yVel, tVel))
        rospy.sleep(0.05)
    publisher.publish(createTwist(0.0, 0.0, 0.0))
    rospy.sleep(0.2)
    publisher.publish(createTwist(0.0, 0.0, 0.0))


def createDemo():
    moveBase(1.4, 0.0, 0.0, createPub)
    moveBase(0.0, 0.0, -2.59, createPub)
    moveBase(1.4, 0.0, 0.0, createPub)
    moveBase(0.0, 0.0, -2.59, createPub)
    # moveBase(0.6, 0.0, 0.0, createPub)
    # moveBase(0.0, 0.0, -1.31, createPub)
    # moveBase(0.35, 0.0, 0.0, createPub)
    # moveBase(0.0, 0.0, -1.31, createPub)
    # moveBase(0.6, 0.0, 0.0, createPub)
    # moveBase(0.0, 0.0, 1.31, createPub)
    # moveBase(0.35, 0.0, 0.0, createPub)
    # moveBase(0.0, 0.0, 1.31, createPub)


def startRosNode():
    rospy.init_node('srt_create')


if __name__ == '__main__':
    startRosNode()
    for i in range(4):
        createDemo()
    # createDemo()
