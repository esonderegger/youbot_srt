#!/usr/bin/env python

import roslib
import rospy
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

roslib.load_manifest('youbot_srt')

toleranceVal = 0.03

# gripper ranges from 0.0 (closed) to 0.0115 (open)

gripperOpenMax = 0.0110
gripperCloseDumpster = 0.003

homeAngles = [0.05, 0.05, -0.05, 0.05, 0.05]
lookForward = [3.0, 0.6, -1.7, 2.9, 2.95]

dumpsterPreApproach = [2.9, 2.4, -4.1, 0.05, 3.0]
dumpsterApproach = [2.95, 1.2, -3.6, 0.05, 3.0]
dumpsterGrasp = [2.95, 0.91, -3.35, 0.05, 3.0]
dumpsterGraspUp = [2.95, 1.28, -3.35, 0.45, 3.0]
dumpsterGraspPreDump = [1.95, 1.28, -3.35, 0.45, 3.0]
dumpsterDump = [1.95, 0.08, -2.85, 2.55, 4.0]

trashCanPreApproach = [3.0, 2.2, -0.8, 0.5, 2.9]
trashCanPreApproach2 = [3.0, 2.50, -1.05, 0.5, 2.94]
trashCanApproach = [3.0, 2.59, -1.09, 0.494, 2.98]
trashCanUp = [3.0, 2.4, -1.4, 0.9, 2.9]
trashCanHigh = [1.1, 1.1, -1.6, 2.2, 2.9]
trashCanPreDump = [0.2, 0.7, -1.5, 2.7, 2.9]
trashCanDump = [0.05, 0.7, -1.3, 2.7, 5.3]

armStarboardUp = [1.4, 2.7, -3.0, 1.0, 3.0]
armStarboardDown = [1.4, 0.55, -3.7, 1.3, 3.0]
armPortUp = [4.3, 0.9, -4.0, 1.4, 3.0]
armPortDown = [4.3, 0.6, -3.9, 1.4, 3.0]
armUp = [3.0, 1.1, -2.5, 1.6, 3.0]


yb_arm = MoveGroupCommander("yb_arm")
yb_gripper = MoveGroupCommander("yb_gripper")
twistPub = rospy.Publisher('cmd_vel', Twist)


def setGripper(amount):
    g = {'gripper_finger_joint_l': amount,
         'gripper_finger_joint_l': amount}
    yb_gripper.set_joint_value_target(g)
    yb_gripper.go()


def poseFromFloats(floatList):
    myPose = Pose()
    myPoint = Point()
    myQuaternion = Quaternion()
    myPoint.x = floatList[0]
    myPoint.y = floatList[1]
    myPoint.z = floatList[2]
    myQuaternion.x = floatList[3]
    myQuaternion.y = floatList[4]
    myQuaternion.z = floatList[5]
    myQuaternion.w = floatList[6]
    myPose.position = myPoint
    myPose.orientation = myQuaternion
    return myPose


def printPose(moveGroup):
    p = moveGroup.get_current_pose('arm_link_5')
    out = [p.pose.position.x]
    out.append(p.pose.position.y)
    out.append(p.pose.position.z)
    out.append(p.pose.orientation.x)
    out.append(p.pose.orientation.y)
    out.append(p.pose.orientation.z)
    out.append(p.pose.orientation.w)
    print out


def moveArmToAngles(moveGroup, anglesList):
    g = {'arm_joint_1': anglesList[0]}
    g['arm_joint_2'] = anglesList[1]
    g['arm_joint_3'] = anglesList[2]
    g['arm_joint_4'] = anglesList[3]
    g['arm_joint_5'] = anglesList[4]
    moveGroup.set_joint_value_target(g)
    moveGroup.go()


def createTwist(xVel, yVel, angVel):
    twist = Twist()
    twist.linear.x = xVel
    twist.linear.y = yVel
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angVel
    return twist



def moveBase(xDist, yDist, theta):
    timeRequired = 0.1
    if abs(xDist / 0.05) > timeRequired:
        timeRequired = abs(xDist / 0.05)
    if abs(yDist / 0.05) > timeRequired:
        timeRequired = abs(yDist / 0.05)
    if abs(theta / 0.1) > timeRequired:
        timeRequired = abs(theta / 0.1)
    stepsRequired = int(timeRequired / 0.1)
    xVel = xDist / timeRequired
    yVel = yDist / timeRequired
    tVel = theta / timeRequired
    twistPub.publish(createTwist(0.0, 0.0, 0.0))
    rospy.sleep(0.3)
    for i in range(stepsRequired):
        twistPub.publish(createTwist(xVel, yVel, tVel))
        rospy.sleep(0.1)
    twistPub.publish(createTwist(0.0, 0.0, 0.0))
    rospy.sleep(0.2)
    twistPub.publish(createTwist(0.0, 0.0, 0.0))
    print 'stopping...'


def goToSleep():
    moveArmToAngles(yb_arm, homeAngles)


def emptyTrashCan():
    moveArmToAngles(yb_arm, lookForward)
    setGripper(0.011)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, trashCanUp)
    moveArmToAngles(yb_arm, trashCanPreApproach)
    moveArmToAngles(yb_arm, trashCanPreApproach2)
    moveArmToAngles(yb_arm, trashCanApproach)
    rospy.sleep(0.2)
    # moveShift(yb_chain, 0, 0.05)
    moveBase(0.05, 0.0, 0.0)
    rospy.sleep(0.2)
    setGripper(0.0025)
    rospy.sleep(1.0)
    moveArmToAngles(yb_arm, trashCanUp)
    moveArmToAngles(yb_arm, trashCanHigh)
    moveArmToAngles(yb_arm, trashCanPreDump)
    moveArmToAngles(yb_arm, trashCanDump)
    rospy.sleep(1.2)
    moveArmToAngles(yb_arm, trashCanPreDump)
    moveArmToAngles(yb_arm, trashCanHigh)
    moveArmToAngles(yb_arm, trashCanUp)
    moveArmToAngles(yb_arm, trashCanApproach)
    rospy.sleep(0.2)
    setGripper(0.011)
    rospy.sleep(0.8)
    # moveShift(yb_chain, 0, -0.05)
    moveBase(-0.05, 0.0, 0.0)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, trashCanPreApproach2)
    moveArmToAngles(yb_arm, trashCanPreApproach)
    moveArmToAngles(yb_arm, lookForward)


def trashCalibrate():
    moveArmToAngles(yb_arm, lookForward)
    setGripper(0.011)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, trashCanUp)
    moveArmToAngles(yb_arm, trashCanPreApproach)
    moveArmToAngles(yb_arm, trashCanPreApproach2)
    moveArmToAngles(yb_arm, trashCanApproach)


def trashUnCalibrate():
    moveArmToAngles(yb_arm, trashCanApproach)
    moveArmToAngles(yb_arm, trashCanPreApproach2)
    moveArmToAngles(yb_arm, trashCanPreApproach)
    moveArmToAngles(yb_arm, trashCanUp)
    setGripper(0.011)
    moveArmToAngles(yb_arm, lookForward)


def emptyDumpster():
    moveArmToAngles(yb_arm, lookForward)
    setGripper(0.011)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, dumpsterPreApproach)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, dumpsterApproach)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, dumpsterGrasp)
    setGripper(0.0025)
    rospy.sleep(1.0)
    moveArmToAngles(yb_arm, dumpsterGraspUp)
    moveArmToAngles(yb_arm, dumpsterGraspPreDump)
    moveArmToAngles(yb_arm, dumpsterDump)
    rospy.sleep(2.0)
    moveArmToAngles(yb_arm, dumpsterGraspPreDump)
    moveArmToAngles(yb_arm, dumpsterGraspUp)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, dumpsterGrasp)
    setGripper(0.011)
    rospy.sleep(0.8)
    moveArmToAngles(yb_arm, dumpsterApproach)
    moveArmToAngles(yb_arm, dumpsterPreApproach)
    moveArmToAngles(yb_arm, lookForward)


def dumpsterCalibrate():
    moveArmToAngles(yb_arm, lookForward)
    setGripper(0.011)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, dumpsterPreApproach)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, dumpsterApproach)
    rospy.sleep(0.2)
    moveArmToAngles(yb_arm, dumpsterGrasp)


def dumpsterUnCalibrate():
    moveArmToAngles(yb_arm, dumpsterApproach)
    moveArmToAngles(yb_arm, dumpsterPreApproach)
    moveArmToAngles(yb_arm, lookForward)


def janitron():
    # moveShift(yb_chain, 1, -0.25)
    moveBase(0.0, -0.25, 0.0)
    # moveShift(yb_chain, 0, 0.25)
    moveBase(-0.25, 0.0, 0.0)
    emptyTrashCan()
    # moveShift(yb_chain, 0, -0.25)
    moveBase(-0.25, 0.0, 0.0)
    # moveShift(yb_chain, 1, -0.50)
    moveBase(0.0, -0.50, 0.0)
    emptyDumpster()
    # moveShift(yb_chain, 1, 0.75)
    moveBase(0.0, 0.75, 0.0)


def startRosNode():
    rospy.init_node('srt_moveit')


if __name__ == '__main__':
    startRosNode()
    # trashCalibrate()
    # moveBase(0.0, 0.7, -0.9)
    # moveBase(0.0, -0.7, 0.9)
    # emptyTrashCan()
    goToSleep()
