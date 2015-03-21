#!/usr/bin/env python
import sys
import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from lane_detection import LaneDetection
from line_detection.cfg import LineDetectionConfig
from itertools import izip

###############################################################################
# Chicago Engineering Design Team
# Brightest Pixel filter using Python OpenCV for autonomous robot Scipio
#    (IGVC competition).
#
# This node finds removes all pixels in every frame except the brightest pixels
# from each row. If the given image was previously filtered properly, this
# should leave us with the lanes on the grass being the brightest, and this
# method would pick these pixels corresponding to the lanes (also reduces the
# amount of data to be converted later into pointclouds).
#
# @author Basheer Subei
# @email basheersubei@gmail.com

# TODO consider using parameters to pick nth brightest pixels instead of one


class BrightestPixel(LaneDetection):
    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    def __init__(self, namespace, node_name):
        LaneDetection.__init__(self, namespace, node_name)

    # this is what gets called when an image is received
    def image_callback(self, ros_image):
        cv2_image = LaneDetection.ros_to_cv2_image(self, ros_image)

        # this filter needs a mono image, no colors
        roi = LaneDetection.convert_to_mono(self, cv2_image)

        roi = LaneDetection.get_roi(self, roi)

        # get indices of max pixels along each row
        # indices = np.argmax(roi, axis=1)

        # get diagonals
        # there are rows-2 + cols-1 diagonals (excluding
        # the stupid bottom-left and top-right single-element corners).
        # test = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12]]
        # roi = np.array([[1, 2, 3, 4, 5, 6], [7, 8, 9, 10, 11, 12]])
        # roi = np.array([[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12]])
        # roi = np.array([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]])
        # roi = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
        # roi = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]]
        num_rows = roi.shape[0]
        num_cols = roi.shape[1]

        num_diagonals = num_rows-2 + num_cols-1
        if num_rows > num_cols:
            low_offset = num_cols - num_rows
            # give the remainder to high offset
            high_offset = num_diagonals - abs(low_offset) - 1
        elif num_rows < num_cols:
            high_offset = num_cols - num_rows
            low_offset = high_offset - num_diagonals + 1
        else:
            low_offset = -num_diagonals//2+1
            high_offset = num_diagonals//2

        diags = []

        print num_rows
        print num_cols
        print num_diagonals
        print low_offset
        print high_offset
        assert high_offset - low_offset + 1 == num_diagonals
        # get indices of max pixels along each diagonal.
        for i in range(low_offset, high_offset+1):
            # print i
            diags.append(list(np.diagonal(roi, offset=i)))
            # print diags
        # print diags
        # print num_rows
        # print num_cols
        # print low_offset
        # print high_offset
        # print diags
        # print len(diags)
        # print len(diags[0])
        # print roi.shape

        diag_values = []
        diag_indices = []

        for diag in diags:
            # print len(diag)
            if len(diag) > 0:  # not an empty list
                # diag_values.append(max(diag))
                diag_indices.append(np.argmax(np.array(diag)))
            else:
                print "HOLY SCHNIKES!"
        brightest_pixels = np.zeros(roi.shape)

        # indexing of elements of original matrix based on diagonal offset and position in diagonal:
        # Given a matrix A and a diagonal offset x, the n-th element of a diagonal corresponds to:
        # if x == 0: A[n][n]
        # if x > 0: A[n][x+n]
        # if x < 0: A[abs(x)+n][n]
        for i, n in enumerate(diag_indices):
            offsets = range(low_offset, high_offset+1)
            x = offsets[i]
            # print roi.shape
            # print x
            # print n
            if x == 0:
                brightest_pixels[n][n] = roi[n][n]
            elif x > 0:
                brightest_pixels[n][x+n] = roi[n][x+n]
            elif x < 0:
                brightest_pixels[abs(x)+n][n] = roi[abs(x)+n][n]

        # rows = np.arange(roi.shape[0])
        # get the values of max pixels along each row

        # values = roi[rows, indices]
        # make an empty image
        # brightest_pixels = np.zeros(roi.shape)

        # now fill the image only with the brightest_pixels from each row

        # for row, (col, pix) in enumerate(izip(indices, values)):
        #     brightest_pixels[row, col] = pix

        final_image = brightest_pixels
        final_image_message = LaneDetection.cv2_to_ros_message(
            self, final_image
        )
        # publishes final image message in ROS format
        self.line_image_pub.publish(final_image_message)
    # end image_callback()


def main(args):
    node_name = "brightest_pixel"
    namespace = rospy.get_namespace()

    # create a BrightestPixel object
    bp = BrightestPixel(namespace, node_name)

    # start the line_detector node and start listening
    rospy.init_node("brightest_pixel", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, bp.reconfigure_callback)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
