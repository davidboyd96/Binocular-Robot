# images is an array of ROS image objects
# H1 is for storing a homography matrix. This needs to be a 3x3 shape for OpenCV use and a 1x9 shape for transmission in this message. 


sensor_msgs/Image[] images
float64[] H1
