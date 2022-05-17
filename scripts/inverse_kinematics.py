#!/usr/bin/env python

import rospy
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kinematicsMessage.srv import kinematicsMessage, kinematicsMessageResponse
try:
    from queue import Queue
except ImportError:
    from Queue import Queue


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
    angles = [0,0,0,0]

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


class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()


def kinematics_server_operation(req):
    # getting joint names
    controller_name = "arm_controller"
    joint_names = rospy.get_param("/%s/joints" % controller_name)
    rospy.loginfo("Joint names: %s" % joint_names)

    # creating command
    trajectory_command = JointTrajectory()
    trajectory_command.header.stamp = rospy.Time.now()
    trajectory_command.joint_names = joint_names
    point = JointTrajectoryPoint()
    joint_angles = inverse_kinematics([req.x, req.y, req.z], req.alpha)
    point.positions = joint_angles
    point.velocities = [0.0, 0.0, 0.0, 0.0]
    point.time_from_start = rospy.rostime.Duration(1, 0)
    trajectory_command.points = [point]

    # publishing command
    pub.publish(trajectory_command)

    # subscriber checking the joints until the gripper arrives to the destination
    error = 0.1
    watchdog = 0
    maxValue = 100000
    actual_joint_angles = qJoints.get()
    while (abs(actual_joint_angles[0] - joint_angles[0]) > error or abs(actual_joint_angles[1] - joint_angles[1]) > error or
           abs(actual_joint_angles[2] - joint_angles[2]) > error or abs(actual_joint_angles[3] - joint_angles[3]) > error)\
            and watchdog < maxValue:
        watchdog = watchdog + 1

    if watchdog >= maxValue:
        retVal = False
    else:
        retVal = True

    # return value
    return retVal


def joint_angles_subscriber(msg):
    qJoints.put([msg[0], msg[1], msg[2], msg[3]])


if __name__ == "__main__":
    queueSize = 1
    qJoints = BufferQueue(queueSize)
    rospy.init_node('send_joint_angles')
    joint_topic = "/joint_topic_placeholder"
    rospy.Subscriber(joint_topic, jointTypePlaceholder, joint_angles_subscriber)
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
    s = rospy.Service('kinematics_handler', kinematicsMessage, kinematics_server_operation)
    rospy.init_node('send_joint_angles_ik')
    rospy.spin()

