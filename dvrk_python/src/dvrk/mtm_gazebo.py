# Start a gazebo using
# > roslaunch rrbot_gazebo rrbot_world.launch

#  Author(s):  Han Lin
#  Created on: 2017-08

import rospy
import threading
import PyKDL
import math
import sys
from std_msgs.msg import String, Bool
from dvrk.arm import *
from dvrk.mtm import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from tf_conversions import posemath
from std_msgs.msg import String, Bool, Float32, Empty
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Quaternion, Wrench, WrenchStamped, TwistStamped
from sensor_msgs.msg import JointState, Joy

import tf

class MTMGazebo(object):
    def __init__(self):
        # data members, event based
        self._robot_name = 'undefined'
        self._robot_state = 'uninitialized'
        self._robot_state_event = threading.Event()
        self._goal_reached = False
        self._goal_reached_event = threading.Event()

        # continuous publish from dvrk_bridge
        self._position_joint_desired = []
        self._position_cartesian_desired = Pose()
        self._model_state = ModelState()
        self._prev_model_state = ModelState()

        self.__position_cartesian_current = PyKDL.Frame()
        self.__twist_body_current = numpy.zeros(6, dtype=numpy.float)
        self.__position_joint_current = numpy.array(0, dtype=numpy.float)
        self.__clutch_event = Joy()
        self.__prev_clutch_event = Joy()

        # publishers
        self.__set_model_state_pub = rospy.Publisher('gazebo/set_model_state',
                                                     ModelState, latch = True, queue_size = 1)

        # subscribers
        rospy.Subscriber('dvrk/footpedals/clutch',
                         Joy, self.__clutch_event_cb)

        self._model_state.model_name = 'mybot'
        self._model_state.reference_frame = 'world'
        self._prev_model_state.model_name = 'mybot'
        self._prev_model_state.reference_frame = 'world'
        self._prev_model_state.pose.position.x = 0
        self._prev_model_state.pose.position.y = 0
        self._prev_model_state.pose.position.z = 0
        self._prev_model_state.pose.orientation.x = 0
        self._prev_model_state.pose.orientation.y = 0
        self._prev_model_state.pose.orientation.z = 0
        self._cord_x = 0
        self._cord_y = 0
        self._cord_z = 0

    def __clutch_event_cb(self, data):
        """Callback for clutch state.

        :param data: the current arm state"""
        self.__clutch_event = data

    def set_model_state(self, arm):
        """set Gazebo model state.

        :param arm: the arm object
        :return: whether or not the model state has been successfully set
        :rtype: Bool"""

        self._model_state.pose = posemath.toMsg(arm.get_current_position())
        self._model_state.pose.position.z = self._model_state.pose.position.z + 0.18

        # quaternion = (
        #     self._model_state.pose.orientation.x,
        #     self._model_state.pose.orientation.y,
        #     self._model_state.pose.orientation.z,
        #     self._model_state.pose.orientation.w)
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[2]
        # yaw = euler[1]
        # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # self._model_state.pose.orientation.x = quaternion[0]
        # self._model_state.pose.orientation.y = quaternion[1]
        # self._model_state.pose.orientation.z = quaternion[2]
        # self._model_state.pose.orientation.w = quaternion[3]

        ori_x_temp = self._model_state.pose.orientation.x
        ori_y_temp = self._model_state.pose.orientation.y
        ori_z_temp = self._model_state.pose.orientation.z
        self._model_state.pose.orientation.x = ori_z_temp
        self._model_state.pose.orientation.y = ori_x_temp
        self._model_state.pose.orientation.z = ori_y_temp

        mtm_twist_body = arm.get_current_twist_body()
        self._model_state.twist.linear.x = mtm_twist_body[0]
        self._model_state.twist.linear.y = mtm_twist_body[1]
        self._model_state.twist.linear.z = mtm_twist_body[2]
        self._model_state.twist.angular.x = mtm_twist_body[3]
        self._model_state.twist.angular.y = mtm_twist_body[4]
        self._model_state.twist.angular.z = mtm_twist_body[5]
        self._prev_model_state.twist.linear.x = mtm_twist_body[0]
        self._prev_model_state.twist.linear.y = mtm_twist_body[1]
        self._prev_model_state.twist.linear.z = mtm_twist_body[2]
        self._prev_model_state.twist.angular.x = mtm_twist_body[3]
        self._prev_model_state.twist.angular.y = mtm_twist_body[4]
        self._prev_model_state.twist.angular.z = mtm_twist_body[5]


        if self.__clutch_event.buttons == []:
            self.__prev_clutch_event.buttons = [0]
            return self.__set_model_state_pub.publish(self._model_state)
        elif self.__clutch_event.buttons[0] - self.__prev_clutch_event.buttons[0] == 1:
            self._prev_model_state.pose.position.x = self._model_state.pose.position.x + self._cord_x
            self._prev_model_state.pose.position.y = self._model_state.pose.position.y + self._cord_y
            self._prev_model_state.pose.position.z = self._model_state.pose.position.z + self._cord_z
            self._prev_model_state.pose.orientation = self._model_state.pose.orientation
            self.__prev_clutch_event = self.__clutch_event
            return self.__set_model_state_pub.publish(self._prev_model_state)
        elif self.__clutch_event.buttons[0] - self.__prev_clutch_event.buttons[0] == -1:
            self._cord_x = self._prev_model_state.pose.position.x - self._model_state.pose.position.x
            self._cord_y = self._prev_model_state.pose.position.y - self._model_state.pose.position.y
            self._cord_z = self._prev_model_state.pose.position.z - self._model_state.pose.position.z
            self.__prev_clutch_event = self.__clutch_event
            return self.__set_model_state_pub.publish(self._prev_model_state)
        elif self.__clutch_event.buttons[0] - self.__prev_clutch_event.buttons[0] == 0:
            if self.__clutch_event.buttons[0] == 1:
                self.__prev_clutch_event = self.__clutch_event
                return self.__set_model_state_pub.publish(self._prev_model_state)
            elif self.__clutch_event.buttons[0] == 0:
                self._model_state.pose.position.x = self._model_state.pose.position.x + self._cord_x
                self._model_state.pose.position.y = self._model_state.pose.position.y + self._cord_y
                self._model_state.pose.position.z = self._model_state.pose.position.z + self._cord_z
                self.__prev_clutch_event = self.__clutch_event
                return self.__set_model_state_pub.publish(self._model_state)

    def get_gripper_state(self, gripper_name):
        """get gripper state of the Gazebo model.

        :param gripper_name: the gripper_name string
        :return: gripper joint state
        :rtype: GetJointProperties"""
        rospy.wait_for_service('/gazebo/get_joint_properties')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
            resp1 = gms(gripper_name)
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_gripper_state(self, gripper_name, gripper_position):
        """set gripper state of the Gazebo model.

        :param gripper_name: the gripper_name string
        :param gripper_position: desired gripper position
        :return: whether or not the gripper state has been successfully set
        :rtype: Bool"""
        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            model_name = 'mybot'
            urdf_param_name = 'robot_description'
            joint_names = gripper_name
            joint_positions = gripper_position
            gms = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
            resp1 = gms(model_name, urdf_param_name, joint_names, joint_positions)
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def mtm_control_gripper(self, arm):
        gripper_position = arm.get_gripper_position()
        if gripper_position < 0:
            gripper_position = 0
        gripper1 = 0.5 * gripper_position# - 1.5708
        if gripper1 > 3.1415926536:
            gripper1 = gripper1 - 3.1415926536 * 2
        elif gripper1 < -3.1415926536:
            gripper1 = gripper1 + 3.1415926536 * 2
        gripper2 = 0.5 * gripper_position# + 1.5708
        if gripper2 > 3.1415926536:
            gripper2 = gripper2 - 3.1415926536 * 2
        elif gripper2 < -3.1415926536:
            gripper2 = gripper2 + 3.1415926536 * 2

        self.set_gripper_state(['jaw_mimic_1', 'jaw_mimic_2'], [gripper1, gripper2])









