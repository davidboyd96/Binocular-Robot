#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import glob
from std_msgs.msg import Int16MultiArray
from random import *


def initCamera(num):
  cap = cv2.VideoCapture()
  cap.open(num)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
  cap.set(cv2.CAP_PROP_FOURCC ,cv2.VideoWriter_fourcc(*'YUYV')) # MJPG for high fps/YUYV for raw
  cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
  return cap

def talker():
    #capture from camera at location 0
    camera_left = initCamera(1)
    camera_right = initCamera(2)
    
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00005)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints_left = [] # 2d points in image plane.
    imgpoints_right = [] # 2d points in image plane.

    '''while True:
        re, img = cap.read()
        cv2.namedWindow("input", flags = cv2.WINDOW_NORMAL)
        cv2.imshow("input", img)
        key = cv2.waitKey(30)
        if key == 27:
            break'''
    matches = 0
    frames = 0
    while matches < 10:
        _ , image_left = camera_left.read()
        _ , image_right = camera_right.read()
        if frames > 50:
            gray_left = cv2.cvtColor(image_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(image_right, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret_left, corners_left = cv2.findChessboardCorners(gray_left, (9,6), None)
            ret_right, corners_right = cv2.findChessboardCorners(gray_right, (9,6), None)
            # If found, add object points, image points (after refining them)
            if ret_left == True and ret_right == True:
                matches += 1
                objpoints.append(objp)   
   
                corners2_left = cv2.cornerSubPix(gray_left,corners_left, (11,11), (-1,-1), criteria)
                corners2_right = cv2.cornerSubPix(gray_right,corners_right, (11,11), (-1,-1), criteria)
                imgpoints_left.append(corners_left)
                imgpoints_right.append(corners_right)

                # Draw and display the corners
                cv2.drawChessboardCorners(image_left, (9,6), corners2_left, ret_left)
                cv2.drawChessboardCorners(image_right, (9,6), corners2_right, ret_right)

        img = np.concatenate((image_left, image_right), axis=1)
        cv2.namedWindow("input", flags = cv2.WINDOW_NORMAL)
        cv2.imshow('input', img)
        key = cv2.waitKey(30)
        frames += 1
        if key == 27:
            break

    _, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(objpoints, imgpoints_left, gray_left.shape[::-1], None, None)
    _, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(objpoints, imgpoints_right, gray_right.shape[::-1], None, None)
    
    print("left:", mtx_left, '\n', dist_left, '\n', rvecs_left, '\n', tvecs_left)
    print("right:", mtx_right, '\n', dist_right, '\n', rvecs_right, '\n', tvecs_right)


    h,  w = image_left.shape[:2]
    newcameramtx_left, roi_left = cv2.getOptimalNewCameraMatrix(mtx_left, dist_left, (w,h), 1, (w,h))
    newcameramtx_right, roi_right = cv2.getOptimalNewCameraMatrix(mtx_right, dist_right, (w,h), 1, (w,h))

    mapx_left, mapy_left = cv2.initUndistortRectifyMap(mtx_left, dist_left, None, newcameramtx_left, (w,h), 5)
    mapx_right, mapy_right = cv2.initUndistortRectifyMap(mtx_right, dist_right, None, newcameramtx_right, (w,h), 5)

    while True:
        _ , image_left = camera_left.read()
        _ , image_right = camera_right.read()
        remap_left = cv2.remap(image_left, mapx_left, mapy_left, cv2.INTER_LINEAR)
        remap_right = cv2.remap(image_right, mapx_right, mapy_right, cv2.INTER_LINEAR)
        img = np.concatenate((remap_left, remap_right), axis=1)
        cv2.imshow('input', img)
        key = cv2.waitKey(30)
        if key == 27:
            break
    camera_left.release()
    camera_right.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
