#!/usr/bin/python

# Author: Brendan Burkhart
# Date: 2022-06-21

# (C) Copyright 2022-2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import cv2
import numpy as np
import crtk
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from cv_bridge import CvBridge


class Camera:
    """
    ROS camera -> OpenCV interface

    Requires calibrated ROS camera
    """

    def __init__(self, ral, info_topic, image_topic):
        """image_topic must be rectified color image"""
        self.ral = ral
        self.cv_bridge = CvBridge()
        self.image_callback = None
        self.camera_matrix = None
        self.camera_frame = None
        self.no_distortion = np.array([], dtype=np.float32)

        self.info_topic = info_topic
        self.image_topic = image_topic

    def set_callback(self, image_callback):
        if self.image_callback is not None and image_callback is not None:
            self.image_callback = image_callback
        elif self.image_callback is not None and image_callback is None:
            self.image_callback = None
        else:
            self.image_callback = image_callback
            self.info_subscriber = self.ral.subscriber(
                self.info_topic, CameraInfo, self._info_callback
            )
            self.image_subscriber = self.ral.subscriber(
                self.image_topic, Image, self._image_callback
            )

    def get_camera_frame(self):
        return self.camera_frame

    def _info_callback(self, info_msg):
        # ROS1 vs ROS2
        if hasattr(info_msg, 'P'):
            projection_matrix = np.array(info_msg.P).reshape((3, 4))
        else:
            projection_matrix = np.array(info_msg.p).reshape((3, 4))
        self.camera_matrix = projection_matrix[0:3, 0:3]
        self.camera_frame = info_msg.header.frame_id

    def _image_callback(self, image_msg):
        callback = self.image_callback  # copy to prevent race
        if self.camera_matrix is None or callback is None:
            return
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        callback(cv_image)

    def project_points(self, object_points, rodrigues_rotation, translation_vector):
        image_points, _ = cv2.projectPoints(
            object_points,
            rodrigues_rotation,
            translation_vector,
            self.camera_matrix,
            self.no_distortion,
        )

        # opencv double-nests the points for some reason, i.e. each point is array([[x, y]])
        image_points = image_points.reshape((-1, 2))

        return image_points

    def get_pose(self, object_points, image_points):
        ok, rotation, translation = cv2.solvePnP(
            object_points, image_points, self.camera_matrix, self.no_distortion
        )
        if not ok:
            return ok, 0.0, rotation, translation

        projected_points = self.project_points(object_points, rotation, translation)
        reprojection_error = np.mean(
            np.linalg.norm(image_points - projected_points, axis=1)
        )

        return ok, reprojection_error, rotation, translation

    def calibrate_pose(self, T_A2B, T_C2D):
        T_A2B_r = np.array([p[0] for p in T_A2B], dtype=np.float64)
        T_A2B_t = np.array([p[1] for p in T_A2B], dtype=np.float64)
        T_C2D_r = np.array([p[0] for p in T_C2D], dtype=np.float64)
        T_C2D_t = np.array([p[1] for p in T_C2D], dtype=np.float64)

        rotation, translation = cv2.calibrateHandEye(
            T_A2B_r,
            T_A2B_t,
            T_C2D_r,
            T_C2D_t,
            method=cv2.CALIB_HAND_EYE_HORAUD,
        )

        def to_homogenous(rotation, translation):
            X = np.eye(4)
            X[0:3, 0:3] = rotation
            X[0:3, 3] = translation.reshape((3,))
            return X

        T_A2B = [to_homogenous(r, t) for r, t in T_A2B]
        T_C2D = [to_homogenous(r, t) for r, t in T_C2D]
        D2A = to_homogenous(rotation, translation)

        constant_transforms = []
        for A2B, C2D in zip(T_A2B, T_C2D):
            T_C2B = T_A2B @ D2A @ T_C2D
            constant_transforms.append(np.linalg.norm(T_C2B, ord="fro"))

        constant_transforms = np.array(constant_transforms)

        error = np.std(constant_transforms - np.mean(constant_transforms))

        return error, rotation, translation
