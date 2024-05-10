#!/usr/bin/env python
import sys
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from nav_msgs.msg import Path
import numpy as np
from gtsam import Rot3
import time

"""
Node to plot trajectory at every PERIOD seconds for visualization
"""

class PathPlotter(object):
   def __init__(self, name_topic_in, rate):

      rospy.loginfo(f"Plotting path from {name_topic_in} at rate of {rate} Hz")
      self.sub = rospy.Subscriber(name_topic_in, Path, self.plot_path,
                                  queue_size=5)
      self.fig = plt.figure()
      self.ax = plt.axes(projection='3d')
      self.prev_plot_time = -1
      self.cur_plot_time = -1
      self.rate = float(rate)
      plt.show()

   def get_position_quat(self, msg):
      position = [msg.pose.position.x,
                  msg.pose.position.y,
                  msg.pose.position.z]
      gtsam_rot3 = Rot3.Quaternion(msg.pose.orientation.w,
                                   msg.pose.orientation.x,
                                   msg.pose.orientation.y,
                                   msg.pose.orientation.z)


      return position, gtsam_rot3.yaw()


   def plot_path(self, cur_msg):
      # Check if now is the current time to plot
      self.cur_plot_time = time.time()
      if self.prev_plot_time < 0 \
         or (1/(self.cur_plot_time - self.prev_plot_time) - self.rate) \
                  < 0.1*self.rate:
         self.prev_plot_time = time.time()
         self.ax.clear()
         all_positions = []
         for pose in cur_msg.poses:
            cur_position, cur_theta = self.get_position_quat(pose)
            all_positions.append(cur_position)
            # self.ax.scatter3D(cur_position[0], cur_position[1], cur_position[2])
         all_positions = np.array(all_positions)
         self.ax.plot(all_positions[:, 0], all_positions[:, 1], all_positions[:, 2])
         plt.draw()

if __name__ == '__main__':
   argv = rospy.myargv(sys.argv)
   if len(argv) == 1:
      argv.append("vio/path_imu")
      argv.append(1)
      print("Usage:")
      print(argv[0] + " name_topic_in")
      print("Plots the path given in input topic, assuming topic type is nav_msgs/Path Message")
      print("Subscribing to default topic -- vio/path_imu, default rate -- 1 Hz")
   elif len(argv) > 3:
      print("Error")
      print("Usage:")
      print(argv[0] + " name_topic_in")
      exit(0)

   rospy.init_node("pathPlotter", anonymous=True)

   gs = PathPlotter(argv[1], argv[2])
   rospy.spin()
