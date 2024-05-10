#!/usr/bin/env python

COMPRESSED = True

import sys
import rospy
if COMPRESSED:
   from sensor_msgs.msg import CompressedImage as ImageImport
else:
   from sensor_msgs.msg import Image as ImageImport
from sensor_msgs.msg import Image as ImageExport
from cv_bridge import CvBridge

if "/opt/ros/kinetic/lib/python2.7/dist-packages" in sys.path:
   sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")

import cv2
import numpy as np

"""
Node to transform an input ImageImport topic into
an output grayscale Image topic.

Author: Adpated from Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

class GrayScaler(object):
   def __init__(self, name_topic_in, name_topic_out):
      self.cv_bridge = CvBridge()
      rospy.loginfo("Converting Images from topic " + name_topic_in +
                 " to grayscale, output topic: " + name_topic_out)
      self.pub = rospy.Publisher(name_topic_out, ImageExport, queue_size=5)
      self.sub = rospy.Subscriber(
         name_topic_in, ImageImport, self.image_cb, queue_size=5)

   def image_cb(self, img_msg):
      # Transform to cv2/numpy ImageImport
      if COMPRESSED:
         img_in_cv2 = self.cv_bridge.compressed_imgmsg_to_cv2(
                        img_msg, desired_encoding='passthrough')
      else:
         img_in_cv2 = self.cv_bridge.imgmsg_to_cv2(
                        img_msg, desired_encoding='passthrough')
      # Transform to grayscale,
      # available encodings: http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html
      if (COMPRESSED and "rgb" in img_msg.format.split(";")[0]) or "rgb" in img_msg.encoding:
         gray_img = cv2.cvtColor(img_in_cv2, cv2.COLOR_RGB2GRAY)
      elif (COMPRESSED and "bgr" in img_msg.format.split(";")[0]) or "bgr" in img_msg.encoding:
         gray_img = cv2.cvtColor(img_in_cv2, cv2.COLOR_BGR2GRAY)

      # Transform back to Image message
      gray_img_mono8 = np.array(gray_img, dtype=np.uint8)
      gray_img_msg = self.cv_bridge.cv2_to_imgmsg(gray_img_mono8,encoding='passthrough')
      # gray_img_msg = self.cv_bridge.cv2_to_imgmsg(gray_img, encoding="mono8")

      gray_img_msg.header.frame_id = "cam0"
      gray_img_msg.header.stamp = img_msg.header.stamp
      self.pub.publish(gray_img_msg)


if __name__ == '__main__':
   argv = rospy.myargv(sys.argv)
   if len(argv) != 3:
      print("Usage:")
      print(argv[0] + " name_topic_in name_topic_out")
      print("Converts a RGB/BGR name_topic_in ImageImport topic into a Grayscale name_topic_out ImageImport topic")
      exit(0)
   rospy.init_node("image_to_grayscale", anonymous=True)

   gs = GrayScaler(argv[1], argv[2])
   rospy.spin()
