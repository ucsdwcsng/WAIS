#!/usr/bin/env python
import sys
import rospy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from gtsam_backend.msg import Aoa
from nav_msgs.msg import Odometry
import numpy as np
from gtsam import Rot3

import pdb

"""
Node to transform data from /odom topic into relative odometery
"""

class AoATester(object):
   def __init__(self, num_aps):

      self.sub_odom = rospy.Subscriber("/odom", Odometry, self.collect_odom,
                                  queue_size=5)
      self.sub_ap_angle = []
      self.sub_bot_angle = []
      self.sub_bot_angle_local = []
      
      self.odom_db = {}
      self.ap_angle_db = {}
      self.bot_angle_db = {}
      self.bot_angle_local_db = {}
      
      for ii in range(num_aps):
         self.ap_angle_db[ii] = {}
         self.bot_angle_db[ii] = {}
         self.bot_angle_local_db[ii] = {}
         self.sub_ap_angle.append(rospy.Subscriber(f"/wifi/ap_angle/ap_{ii}", 
                                                   Aoa, self.collect_aoa,
                                                   (self.ap_angle_db[ii], ), queue_size=5))
         self.sub_bot_angle.append(rospy.Subscriber(f"/wifi/bot_angle/ap_{ii}", 
                                                   Aoa, self.collect_aoa,
                                                   (self.bot_angle_db[ii], ), queue_size=5))
         self.sub_bot_angle_local.append(rospy.Subscriber(f"/wifi/bot_angle_local/ap_{ii}", 
                                                   Aoa, self.collect_aoa,
                                                   (self.bot_angle_local_db[ii], ), queue_size=5))


   def get_position_quat(self, msg):
      position = np.array([msg.pose.pose.position.x,
                           msg.pose.pose.position.y,
                           msg.pose.pose.position.z])
      gtsam_rot3 = Rot3.Quaternion(msg.pose.pose.orientation.w,
                                   msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z)


      return position, gtsam_rot3.yaw(), gtsam_rot3.matrix()


   def collect_odom(self, cur_msg):
      # Collects CUR_POSITION from Nav_msgs/Odometry 
      cur_position, cur_theta, cur_rotmat = self.get_position_quat(cur_msg)
      self.odom_db[cur_msg.header.stamp] = cur_position
      
   def collect_aoa(self, cur_msg, args):
      # Collects CUR_POSITION from Nav_msgs/Odometry 
      db_to_add = args[0]
      if cur_msg.aoa[0] != np.inf:
         db_to_add[cur_msg.header.stamp] = cur_msg.aoa[0] # collect aoa from 1st Tx antenna only


if __name__ == '__main__':
   # argv = rospy.myargv(sys.argv)
   # if len(argv) != 2:
   #    print("Usage:")
   #    print(argv[0] + " nums_aps")
   #    print("Pass number of AP's in the dataset, assumes topic /wifi/bot_angle/ap_xx, /wifi/bot_angle_local/ap_xx and /wifi/ap_angle/ap_xx")
   #    exit(0)
   rospy.init_node("testAoA", anonymous=True)

   aoaTester = AoATester(5)
   # aoaTester = AoATester(int(argv[1]))
   rospy.spin()
   pdb.set_trace()
   
#%%
# with open('odom_db.pickle', 'rb') as handle:
#     odom_db = pkl.load(handle)
# with open('bot_angle_db.pickle', 'rb') as handle:
#     bot_angle_db = pkl.load(handle)
# with open('bot_angle_local_db.pickle', 'rb') as handle:
#     bot_angle_local_db = pkl.load(handle)
# with open('ap_angle_db.pickle', 'rb') as handle:
#     ap_angle_db = pkl.load(handle)

# def plotter(db, ax, label, flag=0):
#    times = []
#    vals = []
#    for key in db.keys():
#       t = key.to_time()
#       times.append(t)
#       vals.append(flag)
   
#    ax.scatter(times, vals, label=label)
