#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from utils import try_to_publish, convert_imgmsg_to_cv_images
from project.msg import ImageArray

HEIGHT =  rospy.get_param("/HEIGHT")
WIDTH = rospy.get_param("/WIDTH")

bgr_rectified_images_publisher = rospy.Publisher("bgr_rectified", ImageArray, queue_size = 1)

SUB_SUCCESS_STRING = "BGR images received and converted"
SUB_ERROR_STRING = "BGR images error with conversion"
PUB_SUCCESS_STRING = "Rectified images published"
PUB_ERROR_STRING = "Rectified images failed to publish due to bridge error"


SIFT = cv2.xfeatures2d.SIFT_create(0, 5, 0.01, 10, 1.2)# defaults 0, 3, 0.04, 10, 1.6

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=100)
FLANN = cv2.FlannBasedMatcher(index_params,search_params)

'''
Get features using SIFT,
Match them with FLANN,
Find the fundamental matrix,
Find transforms for left and right image,
Apply transform to each image
'''
def rectify_images(image_left, image_right):
    grey_left = cv2.cvtColor( image_left, cv2.COLOR_BGR2GRAY );
    grey_right = cv2.cvtColor( image_right, cv2.COLOR_BGR2GRAY );
    kp1, des1 = SIFT.detectAndCompute(grey_left,None)
    kp2, des2 = SIFT.detectAndCompute(grey_right,None)    
            
    matches = FLANN.knnMatch(des1,des2,k=2)


    good = []
    pts1 = []
    pts2 = []
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.6*n.distance: #decreasing this results in fewer but stronger matches
            good.append(m)
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)

    #This happens when one of the cameras is autofocusing.
    threshold = 100
    if(len(pts1) < threshold or len(pts2) < threshold): 
        rospy.logerr("Error with image rectification. Less than {0} feature matches so recification discarded".format(threshold))
        return False, [], [], []

    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_RANSAC,1,0.99)# default 3.0, 0.99
    pts1 = pts1[mask.ravel()==1]
    pts2 = pts2[mask.ravel()==1]

    ret, H1, H2 = cv2.stereoRectifyUncalibrated(pts1, pts2, F, (WIDTH, HEIGHT), threshold = 1)
    rectified_left = cv2.warpPerspective(image_left, H1, (WIDTH, HEIGHT))
    rectified_right = cv2.warpPerspective(image_right, H2, (WIDTH, HEIGHT))

    H1 = H1.reshape((1,9))
    return True, rectified_left, rectified_right, H1[0].tolist()

def callback(imgmsg):
    #Get captured images
    retval, images, _ = convert_imgmsg_to_cv_images(imgmsg, ["bgr8", "bgr8"], SUB_SUCCESS_STRING, SUB_ERROR_STRING)
    image_left = images[0]
    image_right = images[1]

    if(retval == False):
        rospy.logerr("Error with converting ImageArray message to OpenCV images so rectification returned")
        return

    #Rectify the captured images
    retval, rectified_left, rectified_right, H1 = rectify_images(image_left, image_right)

    #Publish the rectified images
    if(retval):
        images = [image_left, image_right, rectified_left, rectified_right]
        image_formats = ["bgr8","bgr8","bgr8","bgr8"]
        try_to_publish(bgr_rectified_images_publisher, images, image_formats, PUB_SUCCESS_STRING, PUB_ERROR_STRING, left_fundamental_matrix = H1)

if __name__ == "__main__":
    rospy.init_node("rectifyImages")

    bgr_images_subscriber = rospy.Subscriber("bgr_images", ImageArray, callback,queue_size=1)

    try:
        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
