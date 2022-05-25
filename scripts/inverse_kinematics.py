#!/usr/bin/env python

import rospy
import math
import actionlib
import sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from open_manipulator_tools.msg import kinematicsActionAction, kinematicsActionFeedback, kinematicsActionResult

try:
    from queue import Queue
except ImportError:
    from Queue import Queue

JointStates = [0, 0, 0, 0]

def inverse_kinematics(coords, gripper_angle=0):
    '''
    Calculates the joint angles according to the desired TCP coordinate and gripper angle
    :param coords: list, desired [X, Y, Z] TCP coordinates
    :param gripper_angle: float, gripper angle in woorld coordinate system (0 = horizontal, pi/2 = vertical)
    :return: list, the list of joint angles, including the 2 gripper fingers
    '''
    # link lengths
    l1 = 0.128
    l2 = 0.024
    l1c = 0.13023 # ua_link = combined l1 - l2 length
    l3 = 0.124    # fa_link
    l4 = 0.126    # tcp_link

    # base offsets
    x_offset = 0.012
    z_offset = 0.0595 + 0.017

    # joint offsets due to combined l1 - l2
    j1_offset = math.atan(l2/l1)
    j2_offset = math.pi/2.0 + j1_offset # includes +90 degrees offset, too

    # default return list
    angles = [0, 0, 0, 0]

    # Calculate the shoulder pan angle from x and y coordinates
    j0 = math.atan(coords[1]/(coords[0] - x_offset))

    # Re-calculate target coordinated to the wrist joint (x', y', z')
    x = coords[0] - x_offset - l4 * math.cos(j0) * math.cos(gripper_angle)
    y = coords[1] - l4 * math.sin(j0) * math.cos(gripper_angle)
    z = coords[2] - z_offset + math.sin(gripper_angle) * l4

    # Solve the problem in 2D using x" and z'
    x = math.sqrt(y*y + x*x)

    # Let's calculate auxiliary lengths and angles
    c = math.sqrt(x*x + z*z)
    alpha = math.asin(z/c)
    beta = math.pi - alpha
    # Apply law of cosines
    gamma = math.acos((l1c*l1c + c*c - l3*l3)/(2*c*l1c))

    j1 = math.pi/2.0 - alpha - gamma - j1_offset
    j2 = math.acos((l1c*l1c + l3*l3 - c*c)/(2*l1c*l3)) - j2_offset
    delta = math.pi - j2 - gamma - j2_offset

    j3 = math.pi + gripper_angle - beta - delta

    angles[0] = j0
    angles[1] = j1
    angles[2] = -j2
    angles[3] = j3

    return angles


#def kinematics_server_operation(req):


class jointAnglesAction(object):
    _fb = kinematicsActionFeedback()
    _res = kinematicsActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, kinematicsActionAction, execute_cb = self.exec_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('Kinematics action server started...')

    def exec_cb(self, goal):
        rate = rospy.Rate(5)
        success = True
        self._fb.time_elapsed = 0

        rospy.loginfo("%s node is moving the robot to the (%f,%f,%f) coordinates"%(self._action_name, goal.x, goal.y, goal.z))

        # getting joint names
        controller_name = "arm_controller"
        joint_names = rospy.get_param("/%s/joints" % controller_name)
        rospy.loginfo("Joint names: %s" % joint_names)

        # creating command
        trajectory_command = JointTrajectory()
        trajectory_command.header.stamp = rospy.Time.now()
        trajectory_command.joint_names = joint_names
        point = JointTrajectoryPoint()
        joint_angles = inverse_kinematics([goal.x, goal.y, goal.z], goal.alpha)
        point.positions = joint_angles
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start = rospy.rostime.Duration(1, 0)
        trajectory_command.points = [point]

        # publishing command
        pub.publish(trajectory_command)

        # subscriber checking the joints until the gripper arrives to the destination
        error = 0.1
        if simSwitch == "sim":
            maxValue = 100
        else:
            maxValue = 5
        delta = [abs(JointStates[0] - joint_angles[0]), abs(JointStates[1] - joint_angles[1]),abs(JointStates[2] - joint_angles[2]), abs(JointStates[3] - joint_angles[3])]
        while (abs(JointStates[0] - joint_angles[0]) > error or abs(
                JointStates[1] - joint_angles[1]) > error or
               abs(JointStates[2] - joint_angles[2]) > error or abs(
                    JointStates[3] - joint_angles[3]) > error) \
                and self._fb.time_elapsed < maxValue:
            self._fb.time_elapsed = self._fb.time_elapsed + 1
            self._as.publish_feedback(self._fb)
            # rospy.loginfo(JointStates)
            # rospy.loginfo(joint_angles)
            # rospy.loginfo(delta)
            rate.sleep()

        if success:
            if self._fb.time_elapsed >= maxValue:
                retVal = True
            else:
                if simSwitch == "sim":
                    retVal = False
                else:
                    retVal = True
            self._res.result = retVal
            rospy.loginfo("%s reached its goal" % self._action_name)
            self._as.set_succeeded(self._res)


def joint_angles_subscriber(msg):
    global JointStates
    if simSwitch == "sim":
        JointStates = [msg.position[2], msg.position[3], msg.position[4], msg.position[5]]
    else:
        JointStates = [msg.position[0], msg.position[1], msg.position[2], msg.position[3]]


if __name__ == "__main__":
    try:
        simSwitch = rospy.myargv(argv=sys.argv)[1]
    except:
        simSwitch = ""
    queueSize = 1
    rospy.init_node('send_joint_angles_ik')
    joint_topic = "/joint_states"
    rospy.Subscriber(joint_topic, JointState, joint_angles_subscriber)
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
    server = jointAnglesAction(rospy.get_name())
    #server = rospy.Service('kinematics_handler', kinematicsMessage, kinematics_server_operation)
    rospy.spin()

