#!/usr/bin/env python3

# Author: Juan Antonio Barragan
# Date: 2024-04-19

# (C) Copyright 2022-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import sys
import time
import argparse
import pathlib
import json
from dataclasses import dataclass, field
import numpy
from scipy.spatial.transform import Rotation

import cv_bridge
import cv2

import crtk
import sensor_msgs.msg

import dvrk_camera_registration


@dataclass
class ImageSubscriber:
    ral: crtk.ral
    camera_image_topic: str
    camera_info_topic: str
    current_frame: numpy.ndarray = field(default=None, init=False)
    camera_matrix: numpy.ndarray = field(default=None, init=False)

    def __post_init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_subscriber = self.ral.subscriber(
            self.camera_image_topic, sensor_msgs.msg.Image, self._img_callback
        )
        self.info_subscriber = self.ral.subscriber(
            self.camera_info_topic, sensor_msgs.msg.CameraInfo, self._info_callback
        )

    def _img_callback(self, data):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError as e:
            # rospy.logerr(e)
            print(e)

    def _info_callback(self, info_msg):
        # ROS1 vs ROS2
        if hasattr(info_msg, 'P'):
            projection_matrix = numpy.array(info_msg.P).reshape((3, 4))
        else:
            projection_matrix = numpy.array(info_msg.p).reshape((3, 4))
        self.camera_matrix = projection_matrix[0:3, 0:3]
        self.camera_frame = info_msg.header.frame_id

    def wait_until_first_frame(self):
        print("Waiting for image topic...")
        timeout = 10
        start = time.time()
        while self.current_frame is None:
            if time.time() - start > timeout:
                raise TimeoutError("Timeout waiting for first frame")

            time.sleep(0.2)


@dataclass
class PoseAnnotator:
    camera_matrix: numpy.ndarray
    cam_T_base: numpy.ndarray
    dist_coeffs: numpy.ndarray = field(
        default_factory=lambda: numpy.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape((-1, 1)),
        init=False,
    )

    def __post_init__(self):
        print(self.dist_coeffs.shape)

    def draw_pose_on_img(self, img: numpy.ndarray, local_measured_cp: numpy.ndarray):
        pose = self.cam_T_base @ local_measured_cp

        tvec = pose[:3, 3]
        rvec = cv2.Rodrigues(pose[:3, :3])[0]

        points_3d = numpy.array([[[0, 0, 0]]], numpy.float32)
        points_2d, _ = cv2.projectPoints(
            points_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )

        points_2d = tuple(points_2d.astype(numpy.int32)[0, 0])

        img = cv2.circle(img, points_2d, 10, (0, 0, 255), -1)
        img = self.draw_axis(img, self.camera_matrix, self.dist_coeffs, pose, size=0.01)

        return img

    def draw_axis(
        self,
        img: numpy.ndarray,
        mtx: numpy.ndarray,
        dist: numpy.ndarray,
        pose: numpy.ndarray,
        size: int = 10,
    ):

        s = size
        thickness = 2
        R, t = pose[:3, :3], pose[:3, 3]
        K = mtx

        rotV, _ = cv2.Rodrigues(R)
        points = numpy.float32([[s, 0, 0], [0, s, 0], [0, 0, s], [0, 0, 0]]).reshape(-1, 3)
        axisPoints, _ = cv2.projectPoints(points, rotV, t, K, dist)
        axisPoints = axisPoints.astype(int)

        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[0].ravel()),
            (255, 0, 0),
            thickness,
        )
        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[1].ravel()),
            (0, 255, 0),
            thickness,
        )

        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[2].ravel()),
            (0, 0, 255),
            thickness,
        )
        return img


def run_pose_visualizer(
    ral: crtk.ral,
    arm_handle: dvrk_camera_registration.ARM,
    camera_image_topic: str,
    camera_info_topic: str,
    cam_T_robot_base: numpy.ndarray,
):

    img_subscriber = ImageSubscriber(ral, camera_image_topic, camera_info_topic)
    img_subscriber.wait_until_first_frame()

    pose_annotator = PoseAnnotator(img_subscriber.camera_matrix, cam_T_robot_base)

    window_name = camera_image_topic
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)

    while not ral.is_shutdown():

        img = img_subscriber.current_frame
        # display control point
        m_cp, _ = arm_handle.local.measured_cp()
        rotation_quaternion = Rotation.from_quat(m_cp.M.GetQuaternion())
        rotation_matrix = numpy.float64(rotation_quaternion.as_matrix())
        local_measured_cp = numpy.eye(4)
        local_measured_cp[0:3, 0:3] = rotation_matrix
        local_measured_cp[0, 3] = m_cp.p[0]
        local_measured_cp[1, 3] = m_cp.p[1]
        local_measured_cp[2, 3] = m_cp.p[2]
        img = pose_annotator.draw_pose_on_img(img, local_measured_cp)
        # display end of shaft
        m_jp, _ = arm_handle.measured_jp()
        jp = m_jp[0:4]
        m_cp = arm_handle.local.forward_kinematics(jp)
        rotation_quaternion = Rotation.from_quat(m_cp.M.GetQuaternion())
        rotation_matrix = numpy.float64(rotation_quaternion.as_matrix())
        local_measured_cp = numpy.eye(4)
        local_measured_cp[0:3, 0:3] = rotation_matrix
        local_measured_cp[0, 3] = m_cp.p[0]
        local_measured_cp[1, 3] = m_cp.p[1]
        local_measured_cp[2, 3] = m_cp.p[2]
        img = pose_annotator.draw_pose_on_img(img, local_measured_cp)
        cv2.imshow(window_name, img)
        k = cv2.waitKey(1)

        if k == 27 or k == ord("q"):
            break

    cv2.destroyAllWindows()


def load_hand_eye_calibration(json_file: pathlib.Path) -> numpy.ndarray:
    with open(json_file, "r") as f:
        data = json.load(f)

    cam_T_robot_base = numpy.array(data['base-frame']['transform']).reshape(4, 4)
    return cam_T_robot_base


def main():
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p",
        "--psm-name",
        type=str,
        required=True,
        choices=["PSM1", "PSM2", "PSM3"],
        help="PSM name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace",
    )
    parser.add_argument(
        "-H",
        "--hand-eye-json",
        type=str,
        required=True,
        help="hand-eye calibration matrix in JSON format using OpenCV coordinate system",
    )
    parser.add_argument(
        "-c",
        "--camera-namespace",
        type=str,
        required=True,
        help="ROS namespace for the camera",
    )
    args = parser.parse_args(argv)

    ral = crtk.ral('vis_gripper_pose')
    if ral.ros_version() == 1:
        camera_image_topic = args.camera_namespace + '/image_rect_color'
    else:
        camera_image_topic = args.camera_namespace + '/image_rect'
    camera_info_topic = args.camera_namespace + '/camera_info'
  
    arm_handle = dvrk_camera_registration.ARM(ral, arm_name=args.psm_name, expected_interval=0.1)
    ral.spin()
    ral.check_connections()
    cv2.setNumThreads(2)
    cam_T_robot_base = load_hand_eye_calibration(args.hand_eye_json)
    run_pose_visualizer(ral, arm_handle, camera_image_topic, camera_info_topic, cam_T_robot_base)


if __name__ == "__main__":
    main()
