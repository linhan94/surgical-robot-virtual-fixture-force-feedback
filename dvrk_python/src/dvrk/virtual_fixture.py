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
from tf import transformations
from tf_conversions import posemath
from std_msgs.msg import String, Bool, Float32, Empty
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Quaternion, Wrench, WrenchStamped, TwistStamped
from sensor_msgs.msg import JointState, Joy

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import trimesh

class VirtualFixture(object):
    def __init__(self, arm):
        """Constructor.  This initializes a few data members.It
        requires a arm object to get DVRK arm data and a gazebo
        object to get gazebo model data"""
        self._current_wrench = Wrench()
        self._get_wrench = arm.get_current_wrench_body()
        self._current_wrench.force.x = self._get_wrench[0]
        self._current_wrench.force.y = self._get_wrench[1]
        self._current_wrench.force.z = self._get_wrench[2]
        self._current_wrench.torque.x = self._get_wrench[3]
        self._current_wrench.torque.y = self._get_wrench[4]
        self._current_wrench.torque.z = self._get_wrench[5]

        self._model_state = ModelState()
        self.__position_cartesian_current = ModelState()
        self.__position_cartesian_prev = ModelState()
        self.__prev_error = 0.0
        self.__twist_body_current = numpy.zeros(6, dtype=numpy.float)
        self.__max_force = 0.0
        self.__penetrate_flag = [0, 0, 0]

        self.__gazebo_model_pose = numpy.zeros(3, dtype=numpy.float)
        self._model_state = self.get_model_state('mybot')
        self.__gazebo_model_pose[0] = self._model_state.pose.position.x
        self.__gazebo_model_pose[1] = self._model_state.pose.position.y
        self.__gazebo_model_pose[2] = self._model_state.pose.position.z
        self.__mesh = trimesh.load_mesh('/home/davinci/catkin_ws/src/gazebo_ros_demos-kinetic-devel/rrbot_description/'
                                 'meshes/heart.stl')

    def get_model_state(self, model_name):
        """get gripper state of the Gazebo model.

        :param model_name: the gripper_name string
        :return: gripper joint state
        :rtype: GetJointProperties"""
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1 = gms(model_name, 'world')
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def cal_virtual_force(self, boundry, flag, arm):
        self.__position_cartesian_current = posemath.toMsg(arm.get_current_position())
        self._get_wrench = arm.get_current_wrench_body()
        self._model_state = self.get_model_state('mybot')
        # self._current_wrench.force.x = self._get_wrench[0]
        # self._current_wrench.force.y = self._get_wrench[1]
        # self._current_wrench.force.z = self._get_wrench[2]
        # self._current_wrench.torque.x = self._get_wrench[3]
        # self._current_wrench.torque.y = self._get_wrench[4]
        # self._current_wrench.torque.z = self._get_wrench[5]
        self.__twist_body_current = arm.get_current_twist_body()
        arm.set_gravity_compensation(False)

        # print(self.__position_cartesian_current)
        # if self.__position_cartesian_current.position.x > (flag * boundry[0]):
        #     self._current_wrench.force.x = 10*(self.__position_cartesian_current.position.x - boundry[0])
        if (self.__position_cartesian_current.position.y > boundry[
                1] and self.__position_cartesian_current.position.y - boundry[1] < 0.06 and self.__penetrate_flag == 0):
            # Dynamic controller
            force = self.dynamic_controller(120, 10, 0, self._model_state.pose.position.y,
                                            boundry[1], self.__twist_body_current[1])
            self.__max_force = force
            if force > 0:
                force = 0
            arm.set_wrench_body_force([0, 0, force])
            #
            # arm.set_wrench_body_force([0, 0, self.pd_controller(100, -0.1, self._model_state.pose.position.y,
            #                                                     boundry[1])])
            #self._current_wrench.force.y = self.pd_controller(0.2, -0.1, self.__position_cartesian_current.position.y, boundry[1])
            # self._current_wrench.force.y = self.dynamic_controller(0.6, 0.15, 0, self.__position_cartesian_current.position.y, boundry[1], self.__twist_body_current[1])
            # arm.set_wrench(self._current_wrench)
        elif self.__position_cartesian_current.position.y - boundry[1] >= 0.06 and self.__penetrate_flag == 0:
            force = self.__max_force + self.dynamic_controller(-80, 10, 0, self._model_state.pose.position.y,
                                            boundry[1], self.__twist_body_current[1])
            # print(force)
            if force > 0:
                force = 0
                self.__penetrate_flag = 1
            arm.set_wrench_body_force([0, 0, force])
        else:
            force = 0
            arm.set_wrench_body_force([0, 0, 0])
            self.__prev_error = 0.0
            # arm.set_gravity_compensation(True)
        # print(arm.get_current_wrench_body())
        # print(self._current_wrench)
        return force

    def cal_3d_guide_virtual_force(self, surface_point, arm):
        temp_position = arm.get_current_position()
        position = [0, 0, 0]
        position[0] = temp_position.p.x()
        position[1] = temp_position.p.y()
        position[2] = temp_position.p.z()
        self._get_wrench = arm.get_current_wrench_body()
        self._model_state = self.get_model_state('mybot')
        self.__twist_body_current = arm.get_current_twist_body()
        arm.set_gravity_compensation(False)
        force = [0.0, 0.0, 0.0]
        for i in range(0, 3):
            if i < 2:
                if 0.005 <= position[i] - surface_point[i] <= 0.06 and self.__penetrate_flag[i] == 0:
                    force[i+1] = self.dynamic_controller(120, 10, 0, self.__gazebo_model_pose[i],
                                                       surface_point[i], self.__twist_body_current[i])
                    self.__penetrate_flag[i] = 1
                elif -0.06 < position[i] - surface_point[i] < 0.005 and self.__penetrate_flag[i] == 0:
                    force[i+1] = self.dynamic_controller(-120, -10, 0, self.__gazebo_model_pose[i],
                                                       surface_point[i], self.__twist_body_current[i])
                    self.__penetrate_flag[i] = 1
                elif self.__penetrate_flag[i] == 1:
                    if position[i] - surface_point[i] > 0.06:
                        force[i+1] = self.dynamic_controller(-80, 10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                        if force[i+1] > 0:
                            force[i+1] = 0
                            self.__penetrate_flag[i] = 2
                    elif 0.005 <= position[i] - surface_point[i] <= 0.06:
                        force[i + 1] = self.dynamic_controller(120, 10, 0, self.__gazebo_model_pose[i],
                                                               surface_point[i], self.__twist_body_current[i])
                    elif position[i] - surface_point[i] < -0.06:
                        force[i+1] = self.dynamic_controller(80, -10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                        if force[i+1] < 0:
                            force[i+1] = 0
                            self.__penetrate_flag[i] = 2
                    elif -0.06 < position[i] - surface_point[i] < 0.005:
                        force[i + 1] = self.dynamic_controller(-120, -10, 0, self.__gazebo_model_pose[i],
                                                               surface_point[i], self.__twist_body_current[i])
                elif self.__penetrate_flag[i] == 2:
                    if -0.05 <= position[i] - surface_point[i] <= 0.05:
                        self.__penetrate_flag[i] = 0
                    else:
                        force[i+1] = 0
                else:
                    force[i+1] = 0
            elif i == 2:
                if 0.005 <= position[i] - surface_point[i] <= 0.06 and self.__penetrate_flag[i] == 0:
                    force[0] = self.dynamic_controller(120, 10, 0, self.__gazebo_model_pose[i],
                                                       surface_point[i], self.__twist_body_current[i])
                    self.__penetrate_flag[i] = 1
                elif -0.06 < position[i] - surface_point[i] < 0.005 and self.__penetrate_flag[i] == 0:
                    force[0] = self.dynamic_controller(-120, -10, 0, self.__gazebo_model_pose[i],
                                                       surface_point[i], self.__twist_body_current[i])
                    self.__penetrate_flag[i] = 1
                elif self.__penetrate_flag[i] == 1:
                    if position[i] - surface_point[i] > 0.06:
                        force[0] = self.dynamic_controller(-80, 10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                        if force[0] > 0:
                            force[0] = 0
                            self.__penetrate_flag[i] = 2
                    elif 0.005 <= position[i] - surface_point[i] <= 0.06:
                        force[0] = self.dynamic_controller(120, 10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                    elif position[i] - surface_point[i] < -0.06:
                        force[0] = self.dynamic_controller(80, -10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                        if force[0] < 0:
                            force[0] = 0
                            self.__penetrate_flag[i] = 2
                    elif -0.06 < position[i] - surface_point[i] < 0.005:
                        force[0] = self.dynamic_controller(-120, -10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                elif self.__penetrate_flag[i] == 2:
                    if -0.05 <= position[i] - surface_point[i] <= 0.05:
                        self.__penetrate_flag[i] = 0
                    else:
                        force[0] = 0
                else:
                    force[0] = 0
        arm.set_wrench_body_force(force)

    def cal_3d_forbidden_virtual_force(self, surface_point, arm):
        temp_position = arm.get_current_position()
        position = [0, 0, 0]
        position[0] = temp_position.p.x()
        position[1] = temp_position.p.y()
        position[2] = temp_position.p.z()
        self._get_wrench = arm.get_current_wrench_body()
        self._model_state = self.get_model_state('mybot')
        self.__twist_body_current = arm.get_current_twist_body()
        arm.set_gravity_compensation(False)
        force = [0.0, 0.0, 0.0]
        for i in range(1, 2):
            if i < 2:
                if 0 <= position[i] - surface_point[i] <= 0.06 and self.__penetrate_flag[i] == 0:
                    force[i+1] = self.dynamic_controller(120, 10, 0, self.__gazebo_model_pose[i],
                                                       surface_point[i], self.__twist_body_current[i])
                    self.__penetrate_flag[i] = 1
                elif 0 <= position[i] - surface_point[i] <= 0.06 and self.__penetrate_flag[i] == 2:
                    force[i+1] = self.dynamic_controller(-120, -10, 0, self.__gazebo_model_pose[i],
                                                       surface_point[i], self.__twist_body_current[i])
                    self.__penetrate_flag[i] = 1
                elif self.__penetrate_flag[i] == 1:
                    if position[i] - surface_point[i] > 0.06:
                        force[i+1] = self.dynamic_controller(-80, 10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                        if force[i+1] > 0:
                            force[i+1] = 0
                            self.__penetrate_flag[i] = 2
                    elif 0.005 <= position[i] - surface_point[i] <= 0.06:
                        force[i + 1] = self.dynamic_controller(120, 10, 0, self.__gazebo_model_pose[i],
                                                               surface_point[i], self.__twist_body_current[i])
                    elif position[i] - surface_point[i] < -0.06:
                        force[i+1] = self.dynamic_controller(80, -10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                        if force[i+1] < 0:
                            force[i+1] = 0
                            self.__penetrate_flag[i] = 2
                    elif -0.06 < position[i] - surface_point[i] < 0.005:
                        force[i + 1] = self.dynamic_controller(-120, -10, 0, self.__gazebo_model_pose[i],
                                                               surface_point[i], self.__twist_body_current[i])
                elif self.__penetrate_flag[i] == 2:
                    if -0.05 <= position[i] - surface_point[i] <= 0.05:
                        self.__penetrate_flag[i] = 0
                    else:
                        force[i+1] = 0
                else:
                    force[i+1] = 0
            elif i == 2:
                if 0.005 <= position[i] - surface_point[i] <= 0.06 and self.__penetrate_flag[i] == 0:
                    force[0] = self.dynamic_controller(120, 10, 0, self.__gazebo_model_pose[i],
                                                       surface_point[i], self.__twist_body_current[i])
                    self.__penetrate_flag[i] = 1
                elif -0.06 < position[i] - surface_point[i] < 0.005 and self.__penetrate_flag[i] == 0:
                    force[0] = self.dynamic_controller(-120, -10, 0, self.__gazebo_model_pose[i],
                                                       surface_point[i], self.__twist_body_current[i])
                    self.__penetrate_flag[i] = 1
                elif self.__penetrate_flag[i] == 1:
                    if position[i] - surface_point[i] > 0.06:
                        force[0] = self.dynamic_controller(-80, 10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                        if force[0] > 0:
                            force[0] = 0
                            self.__penetrate_flag[i] = 2
                    elif 0.005 <= position[i] - surface_point[i] <= 0.06:
                        force[0] = self.dynamic_controller(120, 10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                    elif position[i] - surface_point[i] < -0.06:
                        force[0] = self.dynamic_controller(80, -10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                        if force[0] < 0:
                            force[0] = 0
                            self.__penetrate_flag[i] = 2
                    elif -0.06 < position[i] - surface_point[i] < 0.005:
                        force[0] = self.dynamic_controller(-120, -10, 0, self.__gazebo_model_pose[i],
                                                           surface_point[i], self.__twist_body_current[i])
                elif self.__penetrate_flag[i] == 2:
                    if -0.05 <= position[i] - surface_point[i] <= 0.05:
                        self.__penetrate_flag[i] = 0
                    else:
                        force[0] = 0
                else:
                    force[0] = 0
        arm.set_wrench_body_force(force)

    def search_point(self, arm):
        temp_position = arm.get_current_position()
        position = [0, 0, 0]
        position[0] = temp_position.p.x()
        position[1] = temp_position.p.y()
        position[2] = temp_position.p.z()
        [closest_points,
         distances,
         triangle_id] = self.__mesh.nearest.on_surface([position])
        print('Closest points from the surface of mesh:\n{}'.format(closest_points))


    def pd_controller(self, kp, kd, current_p, desired_p):
        """PD controller

        :param kp: p gain
        :param kd: d gain
        :param current_p: current position
        :param desired_p: desired position
        :returns: pd controller output
        :rtype: float"""
        error = desired_p - current_p
        result = kp * error + kd * (error - self.__prev_error)
        self.__prev_error = error
        return result

    def dynamic_controller(self, s_gain, d_gain, f_bias, current_p, desired_p, velo):
        """controller based on 

        :param s_gain: position stiffness gain
        :param d_gain: position damping gain
        :param f_bias: force bias
        :param current_p: current position
        :param desired_p: desired position
        :param velo: velocity of MTM
        :returns: dynamic controller output
        :rtype: float"""
        error = desired_p - current_p
        result = f_bias + error * s_gain + d_gain * velo
        return result


    def draw_back_method(self, prev_joint, cur_position, boundry, position):
        if cur_position.position.y > boundry[1]:
            joint = prev_joint
            return joint


