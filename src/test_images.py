#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from utils import try_to_publish, convert_imgmsg_to_cv_images
from project.msg import ImageArray



bgr_rectified_images_publisher = rospy.Publisher("bgr_rectified", ImageArray, queue_size = 1)

SUB_SUCCESS_STRING = "BGR images received and converted"
SUB_ERROR_STRING = "BGR images error with conversion"
PUB_SUCCESS_STRING = "Rectified images published"
PUB_ERROR_STRING = "Rectified images failed to publish due to bridge error"

TEST_IMAGES_LOCATION = rospy.get_param("/test_images_location")


def publish_test_images():
    #Load test images
    image_left = cv2.imread(TEST_IMAGES_LOCATION + "left4.png")
    image_right = cv2.imread(TEST_IMAGES_LOCATION + "right4.png")
    rospy.set_param("/WIDTH", image_left.shape[1])
    rospy.set_param("/HEIGHT", image_left.shape[0])
    
    lr = np.concatenate((image_left,image_right), axis=1)
    cv2.namedWindow("Test", cv2.WINDOW_NORMAL)


    #Publish the test images
    images = [image_left, image_right, image_left, image_right]
    image_formats = ["bgr8","bgr8","bgr8","bgr8"]
    rate = rospy.Rate(0.5) # 1hz
    while not rospy.is_shutdown():
        try_to_publish(bgr_rectified_images_publisher, images, image_formats, PUB_SUCCESS_STRING, PUB_ERROR_STRING)
        cv2.imshow('Test', lr)
        cv2.waitKey(100)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("testImages")

  

    try:
        publish_test_images()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
