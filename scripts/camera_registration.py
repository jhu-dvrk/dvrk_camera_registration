#!/usr/bin/env python3

# Author: Brendan Burkhart
# Date: 2022-06-16

# (C) Copyright 2022-2025 Johns Hopkins University (JHU), All Rights Reserved.

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
from dvrk_camera_registration import ARM
from dvrk_camera_registration import convex_hull
from dvrk_camera_registration import vision_tracking


class CameraRegistrationApplication:
    def __init__(self, ral, collection_mode, backup, psm_name, ecm_name, marker_size, expected_interval, camera, replay_file):
        self.ral = ral
        self.collection_mode = collection_mode
        self.replay_file = replay_file
        self.backup = backup
        self.camera = camera
        self.marker_size = marker_size
        self.expected_interval = expected_interval
        self.psm_name = psm_name
        self.psm = ARM(ral, arm_name=psm_name, expected_interval=expected_interval)
        if ecm_name is not None:
            self.ecm = ARM(ral, arm_name=ecm_name, expected_interval=expected_interval)
        else:
            self.ecm = None

    def setup(self):
        self.messages.info('Enabling {}...'.format(self.psm.name))
        if not self.psm.enable(5):
            self.messages.error(
                'Failed to enable {} within 10 seconds'.format(self.psm.name)
            )
            return False

        self.messages.info('Homing {}...'.format(self.psm.name))
        if not self.psm.home(10):
            self.messages.error(
                'Failed to home {} within 10 seconds'.format(self.psm.name)
            )
            return False

        self.messages.info('Homing complete\n')

        return True


    def collect_poses_manually(self):
        poses = []

        self.messages.info('Press "Enter" to add the current pose, or type "d" to stop collection')
        self.done = False
        self.enter = False
        while not self.done:
            if self.enter:
                self.enter = False
                current_pose, _ = self.psm.measured_jp()
                poses.append(current_pose)
                self.messages.info(f'Total poses collected: {len(poses)}')

            time.sleep(self.expected_interval)

        return poses


    def collect_poses_automatically(self):

        self.messages.info('Move the tool around, poses will be added automatically. When done, press "Enter"')

        # make list sparser by ensuring >2mm separation
        euclidean = lambda x: numpy.array(
            [math.sin(x[0]) * x[2], math.sin(x[1]) * x[2], math.cos(x[2])]
        )
        distance = lambda a, b: numpy.linalg.norm(euclidean(a) - euclidean(b))

        poses = []
        self.enter = False
        while self.ok and not self.enter:
            pose, _ = self.psm.measured_jp()
            new = numpy.array([pose[0], pose[1], pose[2]])
            # euclidean distance based on 3 joints if there is already one pose collected
            if len(poses) == 0 or distance(new,
                                           numpy.array((poses[-1][0],
                                                        poses[-1][1],
                                                        poses[-1][2]))) > 0.005:

                ok, _ = self.tracker.acquire_pose(timeout=4.0)

                if ok:
                    poses.append(pose)
                    self.messages.info(f'Total poses collected: {len(poses)}')

            time.sleep(self.expected_interval)

        return poses


    def determine_safe_range_of_motion(self):
        self.messages.info(
            'Release the clutch and move the PSM around to establish the area the PSM can move in.\nIt does not matter if the ArUco tag is visible!'
        )
        self.messages.info('Press "Enter" when done')

        def collect_points(hull_points):
            self.enter = False

            while self.ok and not self.enter:
                pose, _ = self.psm.measured_jp()
                position = numpy.array([pose[0], pose[1], pose[2]])

                # make list sparser by ensuring >2mm separation
                euclidean = lambda x: numpy.array(
                    [math.sin(x[0]) * x[2], math.sin(x[1]) * x[2], math.cos(x[2])]
                )
                distance = lambda a, b: numpy.linalg.norm(euclidean(a) - euclidean(b))
                if len(hull_points) == 0 or distance(position, hull_points[-1]) > 0.005:
                    hull_points.append(position)

                time.sleep(self.expected_interval)

            return hull_points

        hull_points = []

        while True:
            hull_points = collect_points(hull_points)
            if not self.ok:
                return False, None

            hull = convex_hull.convex_hull(hull_points)
            if hull is None:
                self.messages.info('Insufficient range of motion, please continue')
            else:
                break

        # self.messages.info(
        #    'Range of motion displayed in plot, close plot window to continue'
        # )
        # convex_hull.display_hull(hull)
        return self.ok, hull


    # Make sure target is visible and PSM is within range of motion
    def ensure_target_visible(self, safe_range):
        self.enter = True  # run first check immeditately
        first_check = True

        while self.ok:
            time.sleep(0.01)

            if not self.enter:
                continue

            m_jp, _ = self.psm.measured_jp()
            jp = numpy.copy(m_jp)
            visible = self.tracker.is_target_visible(timeout=1)
            in_rom = convex_hull.in_hull(safe_range, jp)

            if not visible:
                self.enter = False
                if first_check:
                    self.messages.warn(
                        '\nPlease position psm so ArUco target is visible, facing towards camera, and roughly centered within camera\'s view\n'
                    )
                    first_check = False
                else:
                    self.messages.warn(
                        'Target is not visible, please re-position. Make sure target is not too close'
                    )
                self.messages.info('Press enter or "d" when enter')
            elif not in_rom:
                self.enter = False
                self.messages.warn(
                    'PSM is not within user supplied range of motion, please re-position'
                )
            else:
                return True, jp

        return False, None


    def collect_data(self, poses):

        target_poses = []
        robot_poses = []

        def measure_pose(joint_pose):
            nonlocal target_poses
            nonlocal robot_poses

            self.psm.move_jp(joint_pose).wait()
            time.sleep(0.5)

            ok, target_pose = self.tracker.acquire_pose(timeout=4.0)
            if not ok:
                return False

            target_poses.append(target_pose)
            self.tracker.display_point(target_poses[-1][1], (255, 255, 0))

            local_m_cp, _ = self.psm.local.measured_cp()
            pose = local_m_cp.Inverse()
            rotation_quaternion = Rotation.from_quat(pose.M.GetQuaternion())
            rotation = numpy.float64(rotation_quaternion.as_matrix())
            translation = numpy.array([pose.p[0], pose.p[1], pose.p[2]], dtype=numpy.float64)

            robot_poses.append((rotation, numpy.array(translation)))

            return True

        def collect(poses, tool_shaft_rotation=math.pi / 8):
            self.messages.progress(0.0)
            idx = 1
            for pose in poses:
                if not self.ok or self.ral.is_shutdown():
                    return

                self.messages.info(f'Collecting data for pose {idx} out of {len(poses)}')
                idx += 1
                shaft_rotations = [
                    math.radians(-60.0),
                    math.radians(-30.0),
                    math.radians(  0.0),
                    math.radians( 30.0),
                    math.radians( 60.0),
                ]
                pose3 = pose[3]

                for shaft_rotation in shaft_rotations:
                    pose[3] = pose3 + shaft_rotation
                    measure_pose(pose)

        if self.backup:
            jdata = { 'poses' : [] }
            for pose in poses:
                jdata['poses'].append(pose.tolist())
            date = datetime.datetime.now().strftime('%Y-%m-%d@%H-%M')
            filename = f'poses-{date}.json'
            with open(filename, 'w') as f:
                f.write(json.dumps(jdata))
            self.messages.info(f'Poses saved in {filename}\n')

        self.messages.info('Collecting pose data...')
        collect(poses)
        self.messages.line_break()
        self.messages.info('Data collection complete\n')

        return robot_poses, target_poses


    # From starting position within view of camera, determine the camera's
    # field of view via exploration while staying within safe range of motion
    # Once field of view is found, collect additional pose samples
    def collect_safe_data(self, safe_range, start_jp, edge_samples=4):
        current_jp = numpy.copy(start_jp)
        current_jp[4:6] = numpy.zeros(2)

        target_poses = []
        robot_poses = []

        def measure_pose(joint_pose):
            nonlocal target_poses
            nonlocal robot_poses

            if not convex_hull.in_hull(safe_range, joint_pose):
                self.messages.error('Safety limit reached!')
                return False

            self.psm.move_jp(joint_pose).wait()
            time.sleep(0.5)

            ok, target_pose = self.tracker.acquire_pose(timeout=4.0)
            if not ok:
                return False

            target_poses.append(target_pose)

            local_m_cp, _ = self.psm.local.measured_cp()
            pose = local_m_cp.Inverse()
            rotation_quaternion = Rotation.from_quat(pose.M.GetQuaternion())
            rotation = numpy.float64(rotation_quaternion.as_matrix())
            translation = numpy.array([pose.p[0], pose.p[1], pose.p[2]], dtype=numpy.float64)

            robot_poses.append((rotation, numpy.array(translation)))

            return True

        def bisect_camera_view(pose, ray, min_steps=4, max_steps=6):
            start_pose = numpy.copy(pose)
            current_pose = numpy.copy(pose)

            far_limit = convex_hull.intersection(safe_range, start_pose[0:3], ray)
            near_limit = 0.0

            for i in range(max_steps):
                if not self.ok:
                    break

                mid_point = 0.5 * (near_limit + far_limit)
                current_pose[0:3] = start_pose[0:3] + mid_point * ray

                ok = measure_pose(current_pose)
                if ok:
                    near_limit = mid_point
                    self.tracker.display_point(target_poses[-1][1], (255, 0, 255))
                else:
                    far_limit = mid_point

                # Only continue past min_steps if we haven't seen target yet
                if i + 1 >= min_steps and near_limit > 0:
                    break

            end_point = start_pose[0:3] + 0.9 * near_limit * ray
            if len(target_poses) > 0:
                self.tracker.display_point(target_poses[-1][1], (255, 123, 66), size=7)

            return end_point

        def collect(poses, tool_shaft_rotation=math.pi / 8.0):
            self.messages.progress(0.0)
            for i, pose in enumerate(poses):
                if not self.ok or self.ral.is_shutdown():
                    return

                rotation_direction = 1 if i % 2 == 0 else -1
                pose[3] = pose[3] + rotation_direction * tool_shaft_rotation
                shaft_rotations = [
                    pose[3] + rotation_direction * tool_shaft_rotation,
                    pose[3] - rotation_direction * tool_shaft_rotation,
                ]

                for shaft_rotation in shaft_rotations:
                    pose[3] = shaft_rotation
                    ok = measure_pose(pose)
                    if ok:
                        self.tracker.display_point(target_poses[-1][1], (255, 255, 0))
                        break

                self.messages.progress((i + 1) / len(sample_poses))

        self.messages.line_break()
        self.messages.info('Determining limits of camera view...')
        self.messages.progress(0.0)
        limits = []

        for axis in range(3):
            ray = numpy.array([0, 0, 0])
            for direction in [1, -1]:
                if not self.ok:
                    return None

                ray[axis] = direction
                limits.append(bisect_camera_view(current_jp, ray))
                self.messages.progress(len(limits) / 6)
        self.messages.line_break()

        # Limits found above define octahedron, take samples along all 12 edges
        sample_poses = []
        for i in range(len(limits)):
            start = i + 2 if i % 2 == 0 else i + 1
            for j in range(start, len(limits)):
                for t in numpy.linspace(
                    1 / (edge_samples + 1), 1 - 1 / (edge_samples + 1), edge_samples
                ):
                    pose = numpy.copy(current_jp)
                    pose[0:3] = limits[j] + t * (limits[i] - limits[j])
                    sample_poses.append(pose)

        self.messages.info('Collecting pose data...')
        collect(sample_poses)
        self.messages.line_break()

        self.messages.info('Data collection complete\n')
        return robot_poses, target_poses

    def compute_registration(self, robot_poses, target_poses):
        error, rotation, translation = self.camera.calibrate_pose(
            robot_poses, target_poses
        )

        if error < 1e-4:
            self.messages.info(
                'Registration error ({:.3e}) is within normal range'.format(error)
            )
        else:
            self.messages.warn(
                'WARNING: registration error ({:.3e}) is unusually high! Should generally be <0.00005'.format(
                    error
                )
            )

        distance = numpy.linalg.norm(translation)
        self.messages.info(
            'Measured distance from RCM to camera origin: {:.3f} m\n'.format(distance)
        )

        return self.ok, rotation, translation

    def save_registration(self, rotation, translation, file_name, dvrk_format):
        rotation = numpy.linalg.inv(rotation)
        translation = -numpy.matmul(rotation, translation)

        transform = numpy.eye(4)
        transform[0:3, 0:3] = rotation
        transform[0:3, 3:4] = translation

        if dvrk_format:
            to_dvrk = numpy.eye(4)
            to_dvrk[0,0] = -to_dvrk[0,0]
            to_dvrk[1,1] = -to_dvrk[1,1]
            transform = to_dvrk @ transform
            if self.ecm:
                ecm_cp, _ = self.ecm.local.measured_cp()
                ecm_transform = numpy.eye(4)
                for i in range(0, 3):
                    ecm_transform[i, 3] = ecm_cp.p[i]
                    for j in range(0, 3):
                        ecm_transform[i, j] = ecm_cp.M[i, j]
                transform = ecm_transform @ transform


        if dvrk_format:
            if self.ecm:
                self.messages.info('The dVRK calibration needs to be copy-pasted to the suj-fixed.json')
                data = {
                    'name': self.psm_name,
                    'measured_cp': transform.tolist(),
                }
                output = json.dumps(data)
            else:
                self.messages.info('The dVRK calibration needs to be copy-pasted to the console-xxx.json')
                data = {
                    'reference-frame': self.tracker.get_camera_frame() or 'camera',
                    'transform': transform.tolist(),
                }
                output = '"base-frame": {}'.format(json.dumps(data))
        else:
            data = {
                'reference-frame': self.tracker.get_camera_frame() or 'camera',
                'transform': transform.tolist(),
            }
            output = '{\n' + '"base-frame": {}'.format(json.dumps(data)) + '\n}'

        with open(file_name, 'w') as f:
            f.write(output)
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
            cv2.setNumThreads(2)
            self.ok = True

            self._init_tracking()
            self.ok = self.ok and self.tracker.start(self._on_d, self._on_enter, self._on_quit)
            if not self.ok:
                return

            self.ok = self.ok and self.setup()
            if not self.ok:
                return

            self.messages.info('Setup finished')

            if self.collection_mode != 'replay':
                self.messages.info('Waiting for data from PSM')
                time.sleep(2.0)

                self.messages.info('Moving to roll center\n')
                measured_jp, _ = self.psm.measured_jp()
                measured_jp[3] = 0.0
                self.psm.move_jp(measured_jp).wait()
                self.messages.info('Loosen and rotate the aruco along the instrument shaft so it faces the camera\nPress "Enter" when done\n')
                self.enter = False
                while self.ok and not self.enter:
                    time.sleep(self.expected_interval)

            data = None

            # collect data manually
            if self.collection_mode == 'manual':
                self.messages.info('Starting manual poses collection')
                poses = self.collect_poses_manually()
                self.messages.info('Collection data based on user poses')
                data = self.collect_data(poses)

            elif self.collection_mode == 'auto':
                self.messages.info('Starting automatic poses collection')
                poses = self.collect_poses_automatically()
                self.messages.info('Collection data based on user trajectory')
                data = self.collect_data(poses)

            elif self.collection_mode == 'replay':
                self.messages.info(f'Loading poses from file {self.replay_file.name}')
                jdata = json.load(self.replay_file)
                poses_list = jdata['poses']
                poses = []
                for pose in poses_list:
                    poses.append(numpy.array(pose))
                data = self.collect_data(poses)

            else:
                ok, safe_range = self.determine_safe_range_of_motion()
                if not self.ok or not ok:
                    return

                ok, start_jp = self.ensure_target_visible(safe_range)
                if not self.ok or not ok:
                    return

                data = self.collect_safe_data(safe_range, start_jp)
                if not self.ok:
                    return

            if len(data[0]) <= 10:
                self.messages.error('Not enough pose data, cannot compute registration')
                self.messages.error(
                    'Please try again, with more range of motion within camera view'
                )
                return

            ok, rvec, tvec = self.compute_registration(*data)
            if not ok:
                return

            self.tracker.stop()

            self.save_registration(
                rvec, tvec, './{}-registration-open-cv.json'.format(self.psm.name), False # using OpenCV frame coordinates
            )

            self.save_registration(
                rvec, tvec, './{}-registration-dVRK.json'.format(self.psm.name), True # using dVRK frame coordinates
            )

        finally:
            self.tracker.stop()


def main():
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-p',
        '--psm-name',
        type=str,
        required=True,
        choices=['PSM1', 'PSM2', 'PSM3'],
        help='PSM name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace',
    )
    parser.add_argument(
        '-m',
        '--marker_size',
        type=float,
        required=True,
        help='ArUco marker side length - including black border - in same units as camera calibration',
    )
    parser.add_argument(
        '-i',
        '--interval',
        type=float,
        default=0.1,
        help='expected interval in seconds between messages sent by the device',
    )
    parser.add_argument(
        '-c',
        '--camera-namespace',
        type=str,
        required=True,
        help='ROS namespace for the camera',
    )
    parser.add_argument(
        '-e',
        '--ecm-name',
        type=str,
        choices=['ECM'],
        help='ECM name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace',
    )
    parser.add_argument('--collection-mode',
                        '-M',
                        type=str,
                        required=True,
                        choices=['manual', 'auto', 'safe', 'replay'],
                        help='manual: move to each pose and press enter\nauto: move around and the code will collect poses as the user moves\nsafe: user moves the arm around to define a safe volume to explore\replay: replay poses saved using -b flag, requires -f',
    )
    parser.add_argument('--backup',
                        '-b',
                        action = 'store_true',
                        help = 'backup poses for later data analysis')
    parser.add_argument('--replay-file',
                        '-r',
                        type = argparse.FileType('r'))

    args = parser.parse_args(argv)

    if args.collection_mode == 'replay' and  args.replay_file is None:
        sys.exit('The replay file has not been specified')

    ral = crtk.ral('dvrk_camera_calibration')

    if ral.ros_version() == 1:
        camera_image_topic = args.camera_namespace + '/image_rect_color'
    else:
        camera_image_topic = args.camera_namespace + '/image_rect'
    camera_info_topic = args.camera_namespace + '/camera_info'

    camera = Camera(ral, camera_info_topic, camera_image_topic)
    application = CameraRegistrationApplication(
        ral,
        collection_mode = args.collection_mode,
        backup = args.backup,
        psm_name = args.psm_name,
        ecm_name = args.ecm_name,
        marker_size = args.marker_size,
        expected_interval = args.interval,
        camera = camera,
        replay_file = args.replay_file
    )
    ral.spin()
    application.run()


if __name__ == '__main__':
    main()
