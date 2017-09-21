#!/usr/bin/env python

#  Author(s):  Han Lin
#  Created on: 2017-08

from mtm_gazebo import *
from virtual_fixture import *
from dvrk.arm import *
from dvrk.mtm import *
from dvrk.psm import *
from dvrk.console import *
import matplotlib.pyplot as plt
import time

if __name__ == '__main__':
    mtm1 = arm('MTMR')
    # mtm1.home()
    mtm1.set_wrench_body_force([0, 0, 0])
    mtm1.set_gravity_compensation(False)
    mtm_gazebo1 = MTMGazebo()
    vf1 = VirtualFixture(mtm1)

    while True:
        try:
            # mtm1.move_joint(jointNum)
            fd_set_model_state = mtm_gazebo1.set_model_state(mtm1)
            # fd_configure = mtm_gazebo1.set_gripper_state(['jaw_mimic_1'], [0.5])
            # fd_configure = mtm_gazebo1.set_gripper_state(['jaw_mimic_2'], [0])
            mtm_gazebo1.mtm_control_gripper(mtm1)
            #mtm1.set_wrench_body_force([0,0,10])
            fd_force = vf1.cal_virtual_force([0, -0.37, 0], [0, -1, 0], mtm1)
            print(fd_force)
            # mtm1.set_gravity_compensation(True)
        except KeyboardInterrupt:
            sys.exit(0)

    # arm1.shutdown()
    # arm1.home()

    #arm1.set_gravity_compensation(True)
    #arm1.set_gravity_compensation(False)
    #arm1.home()