#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from gtsam import Rot3
import gtsam

COMPUTER = "rigatoni"
if COMPUTER == "rigatoni":
   sys.path.append("/home/aarun/Research/p2slam/python/sim_proc") # for graph-helpers
   sys.path.append("/home/aarun/Research/utilities/python") # from plotting-utils
else:
   pass

from graph_helpers import bot, vector3
from plotting_utils import plot_environment

"""
Node to check if the relative odometry node is behaving correcty within
GTSAM.
"""

ODOM_NOISE = gtsam.noiseModel_Diagonal.Sigmas(vector3(0.01, 0.01, 0.1*np.pi/180))
PRIOR_NOISE_BOT = gtsam.noiseModel_Diagonal.Sigmas(vector3(0.1, 0.1, 360*np.pi/180))


class RelativeOdometry(object):
   def __init__(self, relOdom_topic):

      self.sub = rospy.Subscriber(relOdom_topic, PoseStamped, self.add_odom,
                                  queue_size=5)

      # Define new landmark graph and initial estimates
      self.lmrk_graph = gtsam.NonlinearFactorGraph()
      self.lmrk_est = gtsam.Values()
      # Add prior to the first pose
      self.lmrk_graph.add(gtsam.PriorFactorPose2(bot(0), gtsam.Pose2(0, 0, 0),
                                                 PRIOR_NOISE_BOT))
      # Initialize the from and to ID's for the odom factors
      self.fromID = 0
      self.toID = 1
      self.cur_pos = np.array([0., 0., 0.])
      self.all_poses = [self.cur_pos]

      self.lmrk_est.insert(bot(0), gtsam.Pose2(self.cur_pos[0],
                                               self.cur_pos[1],
                                               self.cur_pos[2]))

   def get_position_quat(self, msg):
      position = np.array([msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z])
      gtsam_rot3 = Rot3.Quaternion(msg.pose.orientation.w,
                                   msg.pose.orientation.x,
                                   msg.pose.orientation.y,
                                   msg.pose.orientation.z)


      return position, gtsam_rot3.yaw()


   def add_odom(self, cur_msg):
      # Convert IMG_MSG, of type Nav_msgs/Odometry to relative odometry of type
      # Geometry_msgs/PoseStamped
      del_xy, del_th = self.get_position_quat(cur_msg)
      self.lmrk_graph.add(gtsam.BetweenFactorPose2(bot(self.fromID), bot(self.toID), \
                                  gtsam.Pose2(del_xy[0], del_xy[1], del_th), \
                                  ODOM_NOISE))

      cur_rot = np.array([[np.cos(self.cur_pos[2]), -np.sin(self.cur_pos[2])], \
                          [np.sin(self.cur_pos[2]), np.cos(self.cur_pos[2])]])

      self.cur_pos[:2] += np.squeeze(cur_rot @ del_xy[:2, None])
      self.cur_pos[2] += del_th
      self.lmrk_est.insert(bot(self.toID), gtsam.Pose2(self.cur_pos[0],
                                                       self.cur_pos[1],
                                                       self.cur_pos[2]))

      self.fromID += 1
      self.toID += 1

   def optimize(self):
      graph_params = gtsam.GaussNewtonParams()
      graph_params.setVerbosity('ERROR')
      optimizer = gtsam.GaussNewtonOptimizer(self.lmrk_graph, self.lmrk_est,
                                             graph_params)
      self.lmrk_result = optimizer.optimize()

   def get_results(self,):
      bot_vals1 = []
      bot_vals2 = []
      for var_id in range(self.toID):
         p = self.lmrk_est.atPose2(bot(var_id))
         bot_vals1.append([p.x(), p.y(), p.theta()])

         p = self.lmrk_result.atPose2(bot(var_id))
         bot_vals2.append([p.x(), p.y(), p.theta()])

      return np.array(bot_vals1), np.array(bot_vals2)

   def plot(self,):
      bot_coords_init, bot_coords_final = self.get_results()
      plot_environment(None, bot_coords_final[:, :2], bot_coords_final[:, 2],
                       title='After GTSAM optimized')
      plot_environment(None, bot_coords_init[:, :2], bot_coords_init[:, 2],
                       title='Before GTSAM optimized')


if __name__ == '__main__':
   argv = rospy.myargv(sys.argv)
   if len(argv) != 2:
      print("Usage:")
      print(argv[0] + " relative-odom-topic-name")
      exit(0)
   rospy.init_node("gtsam_testing", anonymous=True)

   gs = RelativeOdometry(argv[1])
   rospy.spin()

   gs.optimize()
   gs.plot()
   plt.show()
