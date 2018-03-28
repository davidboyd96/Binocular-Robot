#!/usr/bin/env python

import traceback
import sys
import os
import tarfile
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from utils import try_to_publish, convert_imgmsg_to_cv_images
from project.msg import ImageArray
os.environ['TF_CPP_MIN_VLOG_LEVEL'] = '5'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '5'
import tensorflow as tf
tf.logging.set_verbosity(5)

 

rospy.init_node("objectRecognition")
MODEL_LOCATION = rospy.get_param("/model_location")

HEIGHT =  rospy.get_param("/HEIGHT")
WIDTH = rospy.get_param("/WIDTH")

SUB_SUCCESS_STRING = "BGR images received and converted"
SUB_ERROR_STRING = "BGR images error with conversion"
PUB_SUCCESS_STRING = "Left rectified image published"
PUB_ERROR_STRING = "Left rectified image failed to publish due to bridge error"

sys.path.append("..")

import label_map_util
import visualization_utils as vis_util

MODEL_NAME = 'frozen_inference_graph.pb'

LABELS_NAME = 'mscoco_label_map.pbtxt'

NUM_CLASSES = 90

detection_graph = tf.Graph()
with detection_graph.as_default():
  print os.getcwd()
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(MODEL_LOCATION + MODEL_NAME, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(MODEL_LOCATION + LABELS_NAME)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

def recognise_objects(image_left, image_right):
    try:
        with detection_graph.as_default():
            with tf.device('/gpu:0'):
                with tf.Session(graph=detection_graph, config=tf.ConfigProto(log_device_placement=True)) as sess:
                    # Definite input and output Tensors for detection_graph
                    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                    # Each box represents a part of the image where a particular object was detected.
                    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                    # Each score represent how level of confidence for each of the objects.
                    # Score is shown on the result image, together with the class label.
                    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
                    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
                    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
    
                    # the array based representation of the image will be used later in order to prepare the
                    # result image with boxes and labels on it.
                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_left_expanded = np.expand_dims(image_left, axis=0)
                    image_right_expanded = np.expand_dims(image_right, axis=0)
                    # Actual detection.
                    (boxes_left, scores_left, classes_left, num_left) = sess.run(
                        [detection_boxes, detection_scores, detection_classes, num_detections],
                        feed_dict={image_tensor: image_left_expanded})
                    (boxes_right, scores_right, classes_right, num_right) = sess.run(
                        [detection_boxes, detection_scores, detection_classes, num_detections],
                        feed_dict={image_tensor: image_right_expanded})
                    # Visualization of the results of a detection.
                    vis_util.visualize_boxes_and_labels_on_image_array(
                        image_left,
                        np.squeeze(boxes_left),
                        np.squeeze(classes_left).astype(np.int32),
                        np.squeeze(scores_left),
                        category_index,
                        use_normalized_coordinates=True,
                        line_thickness=8)
                    vis_util.visualize_boxes_and_labels_on_image_array(
                        image_right,
                        np.squeeze(boxes_right),
                        np.squeeze(classes_right).astype(np.int32),
                        np.squeeze(scores_right),
                        category_index,
                        use_normalized_coordinates=True,
                        line_thickness=8)

                    left_obj_coord = []
                    left_obj_class = []
                    for box, score, class_ in zip(np.squeeze(boxes_left), np.squeeze(scores_left), np.squeeze(classes_left)):
                        if score > 0.5:
                            ymin, xmin, ymax, xmax = box
                            left_obj_coord.append([ (xmin + xmax) / 2, (ymin + ymax) / 2 ])
                            left_obj_class.append(class_)
                            

                    right_obj_coord = []
                    right_obj_class = []
                    for box, score, class_ in zip(np.squeeze(boxes_right), np.squeeze(scores_right), np.squeeze(classes_right)):
                        if score > 0.5:
                            ymin, xmin, ymax, xmax = box
                            right_obj_coord.append([ (xmin + xmax) / 2, (ymin + ymax) / 2 ])
                            right_obj_class.append(class_)

                    cv2.namedWindow("ObjRec", cv2.WINDOW_NORMAL)
                    lr = np.concatenate((image_left, image_right), axis=1)
                    cv2.imshow("ObjRec", lr)
                    cv2.waitKey(10)

                    return True, left_obj_coord, left_obj_class, right_obj_coord, right_obj_class
    except Exception as e:
        print e
        traceback.print_exc()
        rospy.logerr("Error creating point cloud")
    return False, []

def callback(imgmsg):
    #Get captured images
    image_formats = ["bgr8","bgr8"]
    retval, images, _ = convert_imgmsg_to_cv_images(imgmsg, image_formats, SUB_SUCCESS_STRING, SUB_ERROR_STRING)
    image_left = images[0]
    image_right = images[1]

    if(retval == False):
        rospy.logerr("Error with converting ImageArray message to OpenCV images so object recognition returned")
        return

    #Compute disparity
    retval, left_obj_coord, left_obj_class, right_obj_coord, right_obj_class= recognise_objects(image_left, image_right)

    #Publish the left disparity map
    if(retval):
        print "success"
        #try_to_publish(left_disparity_publisher, images, image_formats, PUB_SUCCESS_STRING, PUB_ERROR_STRING)


if __name__ == "__main__":

    bgr_rectified_images_subscriber = rospy.Subscriber("bgr_images", ImageArray, callback, queue_size=1)

    try:
        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
