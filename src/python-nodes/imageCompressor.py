"""OpenCV feature detectors with ros CompressedImage Topics in python.
This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
Code adapted from Simon Haller <simon.haller at uibk.ac.at>

"""

import sys, time
import numpy as np
from scipy.ndimage import filters
from sensor_msgs.msg import CompressedImage
import rospy
import roslib

if "/opt/ros/kinetic/lib/python2.7/dist-packages" in sys.path:
   sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2


class ImageCompressor:

    def __init__(self, topic_in, topic_out):

        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher(topic_in, CompressedImage, queue_size = 5)

        # subscribed Topic
        self.subscriber = rospy.Subscriber(topic_out, CompressedImage, 
                                           self.compressor,  queue_size = 5)


    def compressor(self, ros_data):
        "images are compressed using cv2 to jpeg format"

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        #### Create CompressedIamge ####

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

if __name__ == '__main__':
    print("Potentially buggy code, untested!!")
    argv = rospy.myargv(sys.argv)
    if len(argv) != 3:
       print("Usage:")
       print(argv[0] + " name_topic_in name_topic_out")
       print("Converts a RGB/BGR name_topic_in to compressed name_topic_out")
       exit(0)
    rospy.init_node("image_compressor", anonymous=True)

    gs = ImageCompressor(argv[1], argv[2])
    rospy.spin()

