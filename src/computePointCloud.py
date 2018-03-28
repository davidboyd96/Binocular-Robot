#!/usr/bin/env python

import traceback
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from utils import try_to_publish, convert_imgmsg_to_cv_images
from project.msg import ImageArray


HEIGHT =  rospy.get_param("/HEIGHT")
WIDTH = rospy.get_param("/WIDTH")

SUB_SUCCESS_STRING = "Disparity image received and converted"
SUB_ERROR_STRING = "Disparity image error with conversion"

#Matrices borrowed from ug_stereomatcher calibration files
proj_mat1 = np.array(np.mat("7.3230899280915291e+03 0. 2.4836974544986647e+03 0. ; 0. 7.3035803715514758e+03 1.7170248033347561e+03 0. ; 0. 0. 1. 0."))

proj_mat2 = np.array(np.mat("6.7878081934982329e+03 -1.9217432877870237e+02 3.5255036883483926e+03 -2.0157476822815127e+03; -4.6934583189946494e+01 7.2103539137184798e+03 1.7186641796587207e+03 -9.5797189922230146e+00; -1.4544318745082588e-01 4.8422193563782013e-03 9.8935475545218288e-01 2.2373538998864441e-02"))

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

#lots of nasty data conversion
def compute_point_cloud(left_image, left_disparity):
    try:
        pts1 = np.zeros((2, WIDTH * HEIGHT))
        pts2 = np.zeros((2, WIDTH * HEIGHT))
        for i in range(HEIGHT):
            for j in range(WIDTH): 
                pts1[0][i*WIDTH + j] = j
                pts1[1][i*WIDTH + j] = i
                pts2[0][i*WIDTH + j] = j + left_disparity[i][j]
                pts2[1][i*WIDTH + j] = i

        points = cv2.triangulatePoints(proj_mat1, proj_mat2, pts1, pts2)
        reformatted = np.zeros((HEIGHT, WIDTH, 3))
        for i in range(HEIGHT):
            for j in range(WIDTH): 
                reformatted[i][j][0] = points[0][i*WIDTH + j] / -points[3][i*WIDTH + j]
                reformatted[i][j][1] = points[1][i*WIDTH + j] / -points[3][i*WIDTH + j] 
                reformatted[i][j][2] = points[2][i*WIDTH + j] / -points[3][i*WIDTH + j] 


        colors = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
        mask = left_disparity > left_disparity.min()
        out_points = reformatted[mask]
        out_colors = colors[mask]
        out_fn = 'out.ply'
        write_ply('out.ply', out_points, out_colors)
        print('%s saved' % 'out.ply')
        return True, []
    except Exception as e:
        print e
        traceback.print_exc()
        rospy.logerr("Error creating point cloud")
    return False, []

def callback(imgmsg):
    #Get disparity iamge
    image_formats = ["bgr8","bgr8","bgr8","bgr8","mono8"]
    retval, images, _ = convert_imgmsg_to_cv_images(imgmsg, image_formats, SUB_SUCCESS_STRING, SUB_ERROR_STRING)
    left_image = images[0]
    right_image = images[1]
    rectified_left = images[2]
    rectified_right = images[3]
    left_disparity = np.int16(images[4]) -abs(rospy.get_param("/minimum_disparity"))

    if(retval == False):
        rospy.logerr("Error with converting ImageArray message to OpenCV images so point cloud map creation returned")
        return

    #Compute point cloud
    _, point_cloud = compute_point_cloud(left_image, left_disparity)

    #return value doesn't matter since no publishing is done.


if __name__ == "__main__":
    rospy.init_node("createPointCloud")
    bgr_rectified_images_subscriber = rospy.Subscriber("left_disparity", ImageArray, callback, queue_size=1)

    try:
        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
