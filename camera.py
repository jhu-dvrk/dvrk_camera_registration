#!/usr/bin/python

# Author: Brendan Burkhart 
# Date: 2022-06-21

# (C) Copyright 2022 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import argparse
import cv2
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class Camera:
    """
    ROS camera -> OpenCV interface

    Requires calibrated ROS camera
    """

    def __init__(self, camera_info_topic, image_topic):
        """image_topic must be rectified color image"""
        self.cv_bridge = CvBridge()
        self.image_callback = None
        self.camera_matrix = None
        self.no_distortion = np.array([], dtype=np.float32)

        self.camera_info_topic = camera_info_topic
        self.image_topic = image_topic

    def set_callback(self, image_callback):
        if self.image_callback is not None and image_callback is not None:
            self.image_callback = image_callback
        elif self.image_callback is not None and image_callback is None:
            self.image_subscriber.unregister()
            self.image_subscriber.unregister()
            self.image_callback = None
        else:
            self.image_callback = image_callback
            self.info_subscriber = rospy.Subscriber(self.camera_info_topic, CameraInfo, self._info_callback)
            self.image_subscriber = rospy.Subscriber(self.image_topic, Image, self._image_callback)

    def _info_callback(self, info_msg):
        projection_matrix = np.array(info_msg.P).reshape((3, 4))
        self.camera_matrix = projection_matrix[0:3, 0:3]

    def _image_callback(self, image_msg):
        if self.camera_matrix is None:
            return

        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        self.image_callback(cv_image)

    def project_points(self, object_points, rodrigues_rotation, translation_vector):
        image_points, _ = cv2.projectPoints(object_points, rodrigues_rotation, translation_vector, self.camera_matrix, self.no_distortion)
        
        # opencv double-nests the points for some reason, i.e. each point is array([[x, y]])
        image_points = image_points.reshape((-1, 2))

        return image_points

    def get_pose(self, object_points, image_points):
        ok, rotation, translation = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.no_distortion)
        if not ok:
            return ok, 0.0, rotation, translation

        projected_points = self.project_points(object_points, rotation, translation)
        reprojection_error = np.mean(np.linalg.norm(image_points - projected_points, axis=1))

        return ok, reprojection_error, rotation, translation

    def unregister(self):
        self.info_callback.unregister()
        self.image_callback.unregister()
 