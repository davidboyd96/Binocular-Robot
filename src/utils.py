#!/usr/bin/env python

import traceback
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import message_filters
import numpy as np
from project.msg import ImageArray

#Perform OpenCV to ROS image conversion and try to publish
def try_to_publish(publisher, images, image_formats, success_string, error_string, left_fundamental_matrix=[]):
    bridge = CvBridge()
    msg = ImageArray()
    try:
        rosimgs = []
        for image,image_format in zip(images, image_formats):
            rosimgs.append(bridge.cv2_to_imgmsg(image, image_format))

        msg.images = rosimgs
        msg.H1 = left_fundamental_matrix
        publisher.publish(msg)
        rospy.loginfo(success_string)
    except CvBridgeError as e:
        print(e)
        traceback.print_exc()
        rospy.logerr(error_string)

#Convert to OpenCV images
def convert_imgmsg_to_cv_images(message, image_formats, success_string, error_string ):
    image_array = message.images
    left_fundamental_matrix = message.H1
    cvimages = []
    bridge = CvBridge()
    try:
        for image, image_format in zip(image_array,image_formats):
            cvimages.append(bridge.imgmsg_to_cv2(image, image_format))
        rospy.loginfo(success_string)
        return True, cvimages, left_fundamental_matrix
    except CvBridgeError as e:
        print (e)
        traceback.print_exc()
        rospy.logerr(error_string)
    return False, [], left_fundamental_matrix


