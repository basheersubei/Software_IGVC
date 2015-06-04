#!/usr/bin/env python
import sys
import cv2
import rospy
import math
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig
import numpy as np

###############################################################################
# Chicago Engineering Design Team
# Gabor filter using Python OpenCV for autonomous robot Scipio
# (IGVC competition).
#
# This node runs the Gabor filter on input images and publishes them.
#
# @author Basheer Subei
# @email basheersubei@gmail.com


class Gabor(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)
        self.multiple_filters = rospy.get_param(
            namespace + node_name + "/multiple_filters",
            False
        )

    def build_filters(self):
        filters = []
        for theta in np.arange(0, np.pi, np.pi / self.gabor_number_of_angles):
            # print theta
            kern = cv2.getGaborKernel(
                (self.gabor_ksize, self.gabor_ksize),
                self.gabor_sigma,
                theta,
                self.gabor_lambda,
                self.gabor_gamma
            )
            kern /= 1.5 * kern.sum()
            filters.append(kern)

        return filters

    # this is what gets called when an image is received
    def image_callback(self, ros_image):

        cv2_image = self.ros_to_cv2_image(ros_image)
        # this filter needs a mono image, no colors
        mono = self.convert_to_mono(cv2_image)

        roi = self.get_roi(mono)

        filters = self.build_filters()

        # apply Gabor filter
        accum = np.zeros_like(roi)
        # print filters
        for kern in filters:
            # print kern
            fimg = cv2.filter2D(roi, -1, kern)
            np.maximum(accum, fimg, accum)

        final_image = accum

        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "gabor"
    namespace = rospy.get_namespace()

    # create a gabor object
    g = Gabor(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("gabor", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, g.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
