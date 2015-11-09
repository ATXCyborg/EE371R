#!/usr/bin/env python
import roslib
roslib.load_manifest('trashbot')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class HSVImageTuner:

  window_name = "HSV Value Tuner"
  hue_min = 0
  hue_max = 179
  saturation_min = 0
  saturation_max = 179
  value_min = 0
  value_max = 179
  image = np.zeros((255,255,3), np.uint8)

  def __init__(self):
    cv2.namedWindow(self.window_name)
    cv2.createTrackbar("Hue Minimum", self.window_name, 0, 179, self.set_hueMin)
    cv2.createTrackbar("Hue Maximum", self.window_name, 179, 179, self.set_hueMax)
    cv2.createTrackbar("Saturation Minimum", self.window_name, 0, 255, self.set_saturationMin)
    cv2.createTrackbar("Saturation Maximum", self.window_name, 255,255, self.set_saturationMax)
    cv2.createTrackbar("Value Minimum", self.window_name, 0, 255, self.set_valueMin)
    cv2.createTrackbar("Value Maximum", self.window_name, 255, 255, self.set_valueMax)

  def set_hueMin (self, hue):
    self.hue_min = hue
    self.update_image()

  def set_hueMax (self, hue):
    self.hue_max = hue
    self.update_image()

  def set_saturationMin (self, saturation):
    self.saturation_min = saturation
    self.update_image()

  def set_saturationMax (self, saturation):
    self.saturation_max = saturation
    self.update_image()

  def set_valueMin (self, value):
    self.value_min = value
    self.update_image()

  def set_valueMax (self, value):
    self.value_max = value
    self.update_image()

  def set_image(self, image):
    self.image = image;
    self.update_image()

  def update_image (self):
    upper_limit = np.array([self.hue_max, self.saturation_max, self.value_max])
    lower_limit = np.array([self.hue_min, self.saturation_min, self.value_min])
    mask = cv2.inRange(self.image,lower_limit, upper_limit)
    cv2.imshow(self.window_name, mask)
    cv2.waitKey(3)

class image_converter:
  it = HSVImageTuner()

  def __init__(self):
    cv2.namedWindow("HSV Value Tuner", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image/hsv_image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError, e:
      print e
      return

    self.it.set_image(cv_image)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
