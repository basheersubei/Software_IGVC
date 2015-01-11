#!/usr/bin/env python
import sys
import numpy as np
import cv2
import math
#import time
import rospy
import sensor_msgs
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import rospkg
from dynamic_reconfigure.server import Server
from line_detection.cfg import LineDetectionConfig
from cv_bridge import CvBridge, CvBridgeError

###############################################################################
## Chicago Engineering Design Team
## Line Detection Example using Python OpenCV for autonomous robot Scipio
##    (IGVC competition).
## @author Basheer Subei
## @email basheersubei@gmail.com
#######################################################
##
## Gabor Filter example
##
###############################################################################


class line_detection:

    node_name = "image_averaging_lanedetection"
    namespace = rospy.get_namespace()
    if namespace == "/":
        namespace = ""

    use_mono = rospy.get_param(rospy.get_namespace() + node_name + "/use_mono")
    use_compressed_format = rospy.get_param(rospy.get_namespace() + node_name + "/use_compressed_format")
    subscriber_image_topic = rospy.get_param(rospy.get_namespace() + node_name + "/subscriber_image_topic")
    publisher_image_topic = rospy.get_param(rospy.get_namespace() + node_name + "/publisher_image_topic")
    buffer_size = rospy.get_param(rospy.get_namespace() + node_name + "/buffer_size")
    # this is where we define our variables in the class.
    # these are changed dynamically using dynamic_reconfig and affect
    # the image processing algorithm. A lot of these are not used in the
    # current algorithm.
    image_averaging_frames = 1

    # training_file_name = 'training_for_backprojection_1.png'
    training_file_name = rospy.get_param(rospy.get_namespace() + node_name + "/training_file_name")

    package_path = ''

    image_height = 0
    image_width = 0

    roi_top_left_x = 0
    roi_top_left_y = 0
    roi_width = 2000
    roi_height = 2000

    dilate_size = 0
    dilate_iterations = 0

    def __init__(self):

        # initialize ROS stuff

        # set publisher and subscriber

        # publisher for image of line pixels (only for debugging, not used in
        # map)
        self.output_image_pub = rospy.Publisher( self.namespace +
                                              "/" + self.node_name +
                                              self.publisher_image_topic +
                                              '/compressed',
                                              sensor_msgs.msg.CompressedImage,
                                              queue_size=1)

        # this returns the path to the current package
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('line_detection')

        if self.use_compressed_format:
        # subscriber for ROS image topic
            self.image_sub = rospy.Subscriber(self.subscriber_image_topic +
                                             "/compressed",
                                              CompressedImage, self.image_callback,
                                              queue_size=1, buff_size=self.buffer_size)
        else:
            # use this for uncompressed raw format
            self.image_sub = rospy.Subscriber(self.subscriber_image_topic,
                                           Image,
                                           self.image_callback, queue_size=1,
                                           buff_size=self.buffer_size)


        self.bridge = CvBridge()

        
        # use this if you need to use the camera_info topic (has intrinsic
        #                                                     parameters)
        # self.camera_info_sub = rospy.Subscriber("/camera/camera_info",
        #                                          CameraInfo,
        #                                          self.camera_info_callback,
        #                                          queue_size=1 )
    
    # this is what gets called when an image is recieved
    def image_callback(self, image):

        if(self.use_mono and not self.use_compressed_format and image.encoding != 'mono8'):
            rospy.logerr("image is not mono8! Aborting!")
            return
        
        if self.use_compressed_format:
            #### direct conversion from ROS CompressedImage to CV2 ####
            np_arr = np.fromstring(image.data, np.uint8)
            if self.use_mono:
                img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_GRAYSCALE)
            else:
                img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        else:
            #### direct conversion from ROS Image to CV2 ####
            # first, we need to convert image from sensor_msgs/Image to numpy (or
            # cv2). For this, we use cv_bridge
            try:
                # img = self.bridge.imgmsg_to_cv2(image, "bgr8")
                img = self.bridge.imgmsg_to_cv2(image,
                                                desired_encoding="passthrough")
            except CvBridgeError, e:
                rospy.logerr(e)

        if img is None:
            rospy.logerr("error! img is empty!")
            return

        self.image_height = img.shape[0]
        self.image_width = img.shape[1]

        # if mono, don't take 3rd dimension since there's only one channel
        if self.use_mono:
            roi = img[
            self.roi_top_left_y:self.roi_top_left_y + self.roi_height,
            self.roi_top_left_x:self.roi_top_left_x + self.roi_width,
            ]
        else:
            roi = img[
                self.roi_top_left_y:self.roi_top_left_y + self.roi_height,
                self.roi_top_left_x:self.roi_top_left_x + self.roi_width,
                :
            ]

        # in case roi settings aren't correct, just use the entire image
        if roi.size <= 0:
            rospy.logerr("Incorrect roi settings! Will use the entire image instead!")
            roi = img

        # use entire image as roi (don't cut any parts out)
        # roi = img

        final_image = roi

        #### Create CompressedImage to publish ####
        final_image_message = CompressedImage()
        final_image_message.header.stamp = rospy.Time.now()
        final_image_message.format = "jpeg"
        final_image_message.data = np.array(cv2.imencode(
                                            '.jpg',
                                            final_image)[1]).tostring()

        # publishes image message with line pixels in it
        self.output_image_pub.publish(final_image_message)

    ## end image_callback()

    def reconfigure_callback(self, config, level):

        # TODO check if the keys exist in the config dictionary or else error

        self.image_averaging_frames = config['image_averaging_frames']
        self.validate_parameters()
        return config

    # makes sure the parameters are valid and don't crash the
    # openCV calls. Changes them to valid values if invalid.
    def validate_parameters(self):
        
        # these parameters need validation:
        # now check if ROI parameters are out of bounds
        # only do this if image dimensions have been set
        if self.image_width > 0 and self.image_height > 0:
            if self.roi_width > self.image_width - self.roi_top_left_x:
                self.roi_width = self.image_width - self.roi_top_left_x
            if self.roi_top_left_x < 0:
                self.roi_top_left_x = 0

            if self.roi_height > self.image_height - self.roi_top_left_y:
                self.roi_height = self.image_height - self.roi_top_left_y
            if self.roi_top_left_y < 0:
                self.roi_top_left_y = 0

def main(args):
    # create a line_detection object
    ld = line_detection()

    # start the line_detector node and start listening
    rospy.init_node("line_detection", anonymous=True)

    # starts dynamic_reconfigure server
    srv = Server(LineDetectionConfig, ld.reconfigure_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
