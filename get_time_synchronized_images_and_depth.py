#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.
# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy 
import numpy as np
import message_filters
# Instantiate CvBridge
bridge = CvBridge()
num = 1
def image_callback(msg):
    print("Received an image!")
    global num
    try:
        # Convert your ROS Image message to OpenCV2
        # cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # import numpy as np
        # im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        color_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        str_pth = "./Image_test/image_" + str(num).zfill(6) + ".jpeg"
        cv2.imwrite(str_pth, color_img)
        num += 1
        print("Saved ", str_pth)
def depth_callback(msg):
    print("Received an image!")
    global num
    try:
        # Convert your ROS Image message to OpenCV2
        # cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # import numpy as np
        depth_img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width, -1)
        print(depth_img.shape)
        # color_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        str_pth = "./depth_test/image_" + str(num).zfill(6) + ".npy"
        np.save(str_pth, depth_img)
        num += 1
        print("Saved ", str_pth)

def common_callback(img, depth):
    print("Received an image and a depth!")
    global num
    d_fl, i_fl = False, False
    try:
        depth_img = np.frombuffer(depth.data, dtype=np.uint16).reshape(depth.height, depth.width, -1)
        print(depth_img.shape)
    except CvBridgeError as e:
        print(e)
    else:
        str_pth = "./RGB-Depth/depth_" + str(num).zfill(6) + ".npy"
        np.save(str_pth, depth_img)
        d_fl = True
        # num += 1
        print("Saved ", str_pth)


    try:
        # Convert your ROS Image message to OpenCV2
        # cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # import numpy as np
        # im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        color_img = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        str_pth = "./RGB-Depth/image_" + str(num).zfill(6) + ".jpeg"
        if d_fl==True:
            cv2.imwrite(str_pth, color_img)
            i_fl = True
        # num += 1
        print("Saved ", str_pth)

    if d_fl and i_fl:
        num+=1


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/color/image_raw" #  "/dreamvu/pal/odoa/get/left"
    depth_topic = "/camera/aligned_depth_to_color/image_raw" # Apparently, this is a 16 bit matrix
    # Set up your subscriber and define its callback
    # rospy.Subscriber(depth_topic, Image, depth_callback)
    i_sub = message_filters.Subscriber(image_topic, Image)
    d_sub = message_filters.Subscriber(depth_topic, Image)
    ts = message_filters.TimeSynchronizer([i_sub, d_sub], 10)
    ts.registerCallback(common_callback)
    rospy.spin()

    # Spin until ctrl + c
    rospy.spin()
if __name__ == '__main__':
    main()
