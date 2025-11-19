#  Author(s):  Anton Deguet, Brendan Burkhart
#  Created on: 2022-08-06

#  (C) Copyright 2022-2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import time
import math
import numpy as np
import crtk
import geometry_msgs.msg

class Arm:
    class Local:
        def __init__(self, ral, expected_interval, operating_state_instance):
            self.crtk_utils = crtk.utils(self, ral, expected_interval, operating_state_instance)
            self.crtk_utils.add_measured_cp()
            self.crtk_utils.add_forward_kinematics()

    # initialize the robot
    def __init__(self, ral, arm_name, ros_namespace="", expected_interval=0.01):
        self.ral = ral.create_child(arm_name)
        self.crtk_utils = crtk.utils(self, self.ral, expected_interval)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_measured_js()
        self.crtk_utils.add_measured_cp()
        self.crtk_utils.add_move_jp()

        self.namespace = ros_namespace
        self.local = Arm.Local(self.ral.create_child("local"), expected_interval, operating_state_instance=self)

        base_frame_topic = "/{}/set_base_frame".format(self.namespace)
        self._set_base_frame_pub = self.ral.publisher(
            base_frame_topic, geometry_msgs.msg.PoseStamped, queue_size=1, latch=True
        )

        self.cartesian_insertion_minimum = 0.055
        self.name = arm_name

    # Sets speed ratio for move_cp/move_jp
    def set_speed(self, speed):
        self.trajectory_j_set_ratio(speed)

    def clear_base_frame(self):
        identity = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self._set_base_frame_pub.publish(identity)

    def set_base_frame(self, pose):
        self._set_base_frame_pub.publish(pose)

    # Bring arm back to center
    def center(self):
        pose = np.copy(self.measured_jp())
        pose.fill(0.0)
        pose[2] = self.cartesian_insertion_minimum
        return self.move_jp(pose)

    # Make sure tool is inserted past cannula so move_cp works
    def enter_cartesian_space(self):
        pose = np.copy(self.measured_jp())
        if pose[2] >= self.cartesian_insertion_minimum:

            class NoWaitHandle:
                def wait(self):
                    pass

                def is_busy(self):
                    return False

            return NoWaitHandle()

        pose[2] = self.cartesian_insertion_minimum
        return self.move_jp(pose)
