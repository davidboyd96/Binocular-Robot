#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from utils import try_to_publish
import numpy as np
from project.msg import ImageArray

HEIGHT =  rospy.get_param("/HEIGHT")
WIDTH = rospy.get_param("/WIDTH")

SUCCESS_STRING = "BGR images published"
ERROR_STRING = "BGR images failed to publish due to bridge error"

# function for creating OpenCV camera objects
def initCamera(num, width, height):
  cap = cv2.VideoCapture()
  cap.open(num)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
  cap.set(cv2.CAP_PROP_FOURCC ,cv2.VideoWriter_fourcc(*'YUYV')) # MJPG for high fps/YUYV for raw
  cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
  return cap

def publish_images(cam_left, cam_right):
    framerate = cam_left.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(framerate)

    bgr_images_publisher = rospy.Publisher("bgr_images", ImageArray, queue_size=1)

    while not rospy.is_shutdown():
        if(not cam_left.grab() or not cam_right.grab()):
            rospy.logfatal("Fatal error capturing images. Exiting.")
            break
        #Capture images
        _, image_left = cam_left.retrieve() 
        _, image_right = cam_right.retrieve() 

        #Publish the captured images
        images = [image_left, image_right]
        image_formats = ["bgr8","bgr8"]
        try_to_publish(bgr_images_publisher, images, image_formats, SUCCESS_STRING, ERROR_STRING)


if __name__ == "__main__":
    try:
        rospy.init_node("captureImages")
        camera_left = initCamera(2, WIDTH, HEIGHT)# These may need changed when camera devices are unplugged
        camera_right = initCamera(1, WIDTH, HEIGHT)# These may need changed when camera devices are unplugged
        publish_images(camera_left, camera_right)
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
