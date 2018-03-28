#!/usr/bin/env python

import rospy
import cv2
import tensorflow as tf
from std_msgs.msg import Int16MultiArray
from random import *

def talker():

    cv2.destroyAllWindows()
    pub = rospy.Publisher('servo', Int16MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 0.5hz
    while not rospy.is_shutdown():
        array = [215, 35, 90, 90] # [215, 35, 90, 90] for facing forward
        rospy.loginfo(array)
        pub.publish(data=array)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
