#!/usr/bin/env python
from __future__ import print_function
# import roslib
# roslib.load_manifest('deeplab_resnet')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

'''
      Subscribe Image from camera with ROS and convert to OpenCV format and show the Image!
      Remember to publish image topic!
'''
class ImageSubscriber(object):

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.__cv_image = None

  def callback(self,data):
    try:
      self.__cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # self.imshow()

  # @property
  def get_img(self):
    return self.__cv_image

  def imshow(self):
    if not self.__cv_image is None:
      cv2.imshow("Image window", self.__cv_image)
      cv2.waitKey(3)
    else:
      print('ImageSubscriber: No image received!')
