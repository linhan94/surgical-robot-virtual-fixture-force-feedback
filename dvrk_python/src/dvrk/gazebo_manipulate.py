#!/usr/bin/env python

import inspect
import threading
import math

import rospy
import numpy
import PyKDL

# we should probably not import the symbols and put them in current namespace
from tf import transformations
from tf_conversions import posemath
from std_msgs.msg import String, Bool, Float32, Empty
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Quaternion, Wrench, WrenchStamped, TwistStamped
from sensor_msgs.msg import JointState, Joy
from gazebo_msgs.msg import *


def gazebo1(object):

    def __init__(self,name):
        self.name = name
        self.__init_gazebo1(name)

    def __init_gazebo1(self,name):
        self.name = name
        self.__ros_namespace = '/inter'
        self.__robot_state = 'uninitialized'
        self.__robot_state_event = threading.Event()
        self.__goal_reached = False
        self.__goal_reached_event = threading.Event()

        self.__joint_properties = numpy.array(0, dtype = numpy.float)

        self.__set_joint_properties_pub = rospy.Publisher(self.__ros_namespace
                                                     + '/GetJointProperties',
                                                     String, latch=True, queue_size=1)

        rospy.Subscriber(self.__ros_namespace + '/get_joint_properties',
                         GetJointProperties, self.__joint_properties)

        # create node
        rospy.init_node('my_gazebo', anonymous = True, log_level = rospy.WARN)

    def get_joint_properties(self):
        """Get the current cartesian velocity of the arm.  This
        is based on the body jacobian, both linear and angular are
        rotated to be defined in base frame.

        :returns: the current position of the arm in cartesian space
        :rtype: geometry_msgs.TwistStamped"""
        return self.__joint_properties

    def set_joint_properties(self,abs_joint):
        return self.__set_joint_properties_pub.publish(abs_joint)