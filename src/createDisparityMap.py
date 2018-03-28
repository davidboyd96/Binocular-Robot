#!/usr/bin/env python

import traceback
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from utils import try_to_publish, convert_imgmsg_to_cv_images
from project.msg import ImageArray

rospy.init_node("createDisparity")

HEIGHT =  rospy.get_param("/HEIGHT")
WIDTH = rospy.get_param("/WIDTH")

SUB_SUCCESS_STRING = "Rectified images received and converted"
SUB_ERROR_STRING = "Rectified images error with conversion"
PUB_SUCCESS_STRING = "Left rectified image published"
PUB_ERROR_STRING = "Left rectified image failed to publish due to bridge error"

left_disparity_publisher = rospy.Publisher("left_disparity", ImageArray, queue_size = 1)

block_size = 7
min_disp = rospy.get_param("/minimum_disparity") #SET THIS AS A ROS PARAMETER -32
num_disp = 128
left_stereomatcher = cv2.StereoSGBM_create(
            minDisparity = min_disp,
            numDisparities = num_disp,
            blockSize = block_size,
            uniquenessRatio =15,
            speckleWindowSize = 0,
            speckleRange = 2,
            disp12MaxDiff = 1,
            preFilterCap = 63,
            P1 = 8*3*block_size**2,
            P2 = 32*3*block_size**2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

# FILTER Parameters
lmbda = 16000
sigma = 10

wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_stereomatcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)
right_stereomatcher = cv2.ximgproc.createRightMatcher(left_stereomatcher)


#Compute left and right disparities
#Filter the disparity
#Perform inverse warp

def compute_disparity(image_left, image_right, H1):
    try:
        grey_left = cv2.cvtColor( image_left, cv2.COLOR_BGR2GRAY )
        grey_right = cv2.cvtColor( image_right, cv2.COLOR_BGR2GRAY )

        disparity_left = left_stereomatcher.compute(grey_left,grey_right,cv2.CV_8UC1)
        disparity_right = right_stereomatcher.compute(grey_right,grey_left,cv2.CV_8UC1)
        disparity_left = disparity_left.astype(np.float32) / 16.0
        disparity_right = disparity_right.astype(np.float32) / 16.0
        disparity_left = np.int16(disparity_left)
        disparity_right = np.int16(disparity_right)
        #these lines are used for showing the images
        x = np.array([])
        y = np.array([])
        a = disparity_left
        b = disparity_right
        x = cv2.normalize(src=a, dst=x, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
        x = np.uint8(x)
        y = cv2.normalize(src=b, dst=y, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
        y = np.uint8(y)


        filtered_disparity = wls_filter.filter(disparity_left, image_left, None, disparity_right)

        if(H1 != ()):
            filtered_disparity = cv2.warpPerspective(filtered_disparity, H1, (WIDTH, HEIGHT), flags = cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_CONSTANT, borderValue = 5)
        filtered_disparity += abs(min_disp)
        filtered_disparity = np.uint8(filtered_disparity)

        filteredImg = np.array([])
        filteredImg = cv2.normalize(src=filtered_disparity, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
        filteredImg = np.uint8(filteredImg)
        lr = np.concatenate((x, y), axis = 1)
        lra = np.concatenate((lr, filteredImg), axis=1)
        cv2.namedWindow("Disparity Map", cv2.WINDOW_NORMAL)
        cv2.imshow('Disparity Map', lra)
        cv2.waitKey(10)
        return True, filtered_disparity
    except Exception as e:
        print e
        traceback.print_exc()
        rospy.logerr("Error calculating disparity")
    return False, []

def callback(imgmsg):
    #Get rectified images
    image_formats = ["bgr8","bgr8","bgr8","bgr8"]
    retval, rectified_images, H1 = convert_imgmsg_to_cv_images(imgmsg, image_formats, SUB_SUCCESS_STRING, SUB_ERROR_STRING)
    left_image = rectified_images[0]
    right_image = rectified_images[1]
    rectified_left = rectified_images[2]
    rectified_right = rectified_images[3]
    if(H1 != ()):
        H1 = np.array(H1).reshape((3,3))


    a = np.concatenate((left_image,right_image), axis=1)
    b = np.concatenate((rectified_left,rectified_right), axis=1)
    ab = np.concatenate((a,b), axis=0)
    cv2.namedWindow("All", cv2.WINDOW_NORMAL)
    cv2.imshow('All', ab)
    cv2.waitKey(10)
    if(retval == False):
        rospy.logerr("Error with converting ImageArray message to OpenCV images so disparity map creation returned")
        return

    #Compute disparity
    retval, disparity_left = compute_disparity(rectified_left, rectified_right, H1)

    #Publish the left disparity map
    if(retval):
        images = [left_image, right_image, rectified_left, rectified_right, disparity_left]
        image_formats.append("mono8")
        try_to_publish(left_disparity_publisher, images, image_formats, PUB_SUCCESS_STRING, PUB_ERROR_STRING)


if __name__ == "__main__":
    bgr_rectified_images_subscriber = rospy.Subscriber("bgr_rectified", ImageArray, callback, queue_size=1)

    try:
        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
