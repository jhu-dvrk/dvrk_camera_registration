#!/usr/bin/env python3

# Author: Brendan Burkhart
# Date: 2025-11-17

# (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import argparse
import sys
import time
import datetime
import cv2
import json
import math
import numpy
import json
from scipy.spatial.transform import Rotation

import crtk
from dvrk_camera_registration import Camera
from dvrk_camera_registration import Arm
from dvrk_camera_registration import convex_hull
from dvrk_camera_registration import vision_tracking


class ECMOpticalRegistration:
    def __init__(self, ral, ecm_name: str, marker_size: float, camera):
        self.ral = ral
        self.camera = camera
        self.marker_size = marker_size
        self.ecm = Arm(ral, arm_name=ecm_name, expected_interval=expected_interval)

    def setup(self):
        self.messages.info("Enabling {}...".format(self.psm.name))
        if not self.psm.enable(5):
            self.messages.error(
                "Failed to enable {} within 10 seconds".format(self.psm.name)
            )
            return False

        self.messages.info("Homing {}...".format(self.psm.name))
        if not self.psm.home(10):
            self.messages.error(
                "Failed to home {} within 10 seconds".format(self.psm.name)
            )
            return False

        self.messages.info("Homing complete\n")

        return True

    def collect_poses(self):
        target_poses = []
        robot_poses = []

        self.done = False
        self.enter = False
        while not self.done:
            if self.enter:
                self.enter = False
                current_pose, _ = self.psm.measured_jp()

                local_m_cp, _ = self.ecm.local.measured_cp()
                R = numpy.float64(Rotation.from_quat(pose.M.GetQuaternion()).as_matrix())
                t = numpy.array([pose.p[0], pose.p[1], pose.p[2]], dtype=numpy.float64)

                robot_poses.append((R, t))

                ok, target_pose = self.tracker.acquire_pose(timeout=4.0)
                if not ok:
                    continue

                robot_poses.append(current_pose)
                target_poses.append(target_pose)
                self.messages.info(f'Total poses collected: {len(robot_poses)}')

            time.sleep(self.expected_interval)

        return poses

    def compute_registration(self, robot_poses, target_poses):
        error, transform = self.camera.calibrate_pose(
            robot_poses, target_poses
        )

        distance = numpy.linalg.norm(transform[0:3, 3])
        self.messages.info(
            'Measured distance from robot tip frame to camera optical origin: {:.3f} m\n'.format(distance)
        )

        return self.ok, transform

    def save_registration(self, transform, file_name):
        with open(file_name, 'w') as f:
            f.write(transform.tolist())
            f.write('\n')

        self.messages.info('Hand-eye calibration saved to {}'.format(file_name))

    # Exit key (q/ESCAPE) handler for GUI
    def _on_quit(self):
        self.ok = False
        self.tracker.stop()
        self.messages.info('\nExiting...')

    # Enter handler for GUI
    def _on_enter(self):
        self.enter = True

    # 'd' handler for GUI
    def _on_d(self):
        self.done = True

    def _init_tracking(self):
        target_type = vision_tracking.ArUcoTarget(
            self.marker_size, cv2.aruco.DICT_4X4_50, [0]
        )
        parameters = vision_tracking.VisionTracker.Parameters(4)
        self.messages = vision_tracking.MessageManager()
        self.tracker = vision_tracking.VisionTracker(
            target_type, self.messages, self.camera, parameters
        )

    def run(self):
        print('Checking topics')
        self.ral.check_connections()
        print('Connections with dVRK checked')

        try:
            self.ok = True

            self._init_tracking()
            self.ok = self.ok and self.tracker.start(self._on_d, self._on_enter, self._on_quit)
            if not self.ok:
                return

            self.ok = self.ok and self.setup()
            if not self.ok:
                return

            self.messages.info('Setup finished')
            self.messages.info('Waiting for data from ECM')
            time.sleep(2.0)

            self.enter = False
            self.messages.info('Press "Enter" to add the current pose, or type "d" to stop collection')
            while self.ok and not self.done:
                pose = self.measure_pose()
                time.sleep(self.expected_interval)

            data = self.collect_poses()

            if len(data[0]) <= 10:
                self.messages.error('Not enough pose data, cannot compute registration')
                self.messages.error(
                    'Please try again, with more range of motion within camera view'
                )
                return

            ok, transform = self.compute_registration(*data)
            if not ok:
                self.messages.error("Failed to compute registration")
                return

            self.tracker.stop()

            self.save_registration(
                transform, './{}_optical_registration.json'.format(self.ecm.name)
            )
        finally:
            self.tracker.stop()


def main():
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-e',
        '--ecm-name',
        type=str,
        default="ECM",
        help='ECM name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace',
    )
    parser.add_argument(
        '-m',
        '--marker_size',
        type=float,
        required=True,
        help='ArUco marker side length - including black border - in same units as camera calibration',
    )
    parser.add_argument(
        '-c',
        '--camera-namespace',
        type=str,
        required=True,
        help='ROS namespace for the camera',
    )
    args = parser.parse_args(argv)

    ral = crtk.ral("dvrk_camera_calibration")

    image_topic = "/image_rect_color" if ral.ros_version() == 1 else "/image_rect"
    image_topic = args.camera_namespace + image_topic
    camera_info_topic = args.camera_namespace + '/camera_info'

    camera = Camera(ral, camera_info_topic, image_topic)
    application = ECMOpticalRegistration(
        ral,
        ecm_name = args.ecm_name,
        marker_size = args.marker_size,
        camera = camera
    )
    ral.spin()
    application.run()


if __name__ == '__main__':
    main()
