"""
Extract trajectory predictions from stored bag files into a mat file for easier post-processing
"""

import numpy as np
import rosbag
import scipy.io as sio
import sys
import os
from tqdm import tqdm
import pdb
from sensor_msgs import point_cloud2
import gtsam
import h5py as hio
import matplotlib.pyplot as plt
import scipy.optimize as opt
from graph_helpers import apply_global_rot, correct_bias


#%%
"""
These topics must exists in the bag_dir:
/wifio/path_robot /wifio/pose_ap -- predictions of the robot and AP positions by WAIS
/cpu_monitor/wifio_test1/cpu /cpu_monitor/wifio_test1/mem -- cpu and memory consumptions of WAIS module
/vio/path_robot -- Kimera or Cartographer robot path 
/cpu_monitor/vio_test1/cpu /cpu_monitor/vio_test1/mem -- cpu and memory consumptions of VIO module (note to turn off any mapping and visualization)
"""

   
bag_dir = "bag file path"
saved_mat_file_name = "final_results.mat"
#####################
#####################

input_bag_file = os.path.join(bag_dir)
bag_in = rosbag.Bag(input_bag_file, 'r')

ap_poses = []

wifi_position = [] # realtime traj
wifi_yaw = [] # realtime traj
wifi_time = []

vio_position = [] # realtime traj
vio_yaw= [] # realtime traj
vio_time = []

wifi_full_traj = None
vio_full_traj = None
   
cpu_consumption = {"vio_test1": [], "wifio_test1": []}
mem_consumption = {"vio_test1": [], "wifio_test1": []}
for topic, mesg, t in tqdm(bag_in.read_messages()):
   
   if topic == "/wifio/path_robot":
      wifi_position.append([mesg.poses[-1].pose.position.x, 
                            mesg.poses[-1].pose.position.y, 
                            mesg.poses[-1].pose.position.z])
      cur_yaw = gtsam.Rot3.Quaternion(mesg.poses[-1].pose.orientation.w, 
                                      mesg.poses[-1].pose.orientation.x, 
                                      mesg.poses[-1].pose.orientation.y, 
                                      mesg.poses[-1].pose.orientation.z).yaw()
      cur_time = mesg.poses[-1].header.stamp.secs + \
                  mesg.poses[-1].header.stamp.nsecs*1e-9
      wifi_yaw.append(cur_yaw)
      wifi_time.append(cur_time)
      wifi_full_traj = mesg.poses
   elif topic == "/wifio/pose_ap" and len(mesg.points):
      ap_poses.append([[p.x, p.y, p.z] for p in mesg.points])
   elif topic == "/vio/path_robot":
      vio_position.append([mesg.poses[-1].pose.position.x, 
                            mesg.poses[-1].pose.position.y, 
                            mesg.poses[-1].pose.position.z])
      cur_yaw = gtsam.Rot3.Quaternion(mesg.poses[-1].pose.orientation.w, 
                                      mesg.poses[-1].pose.orientation.x, 
                                      mesg.poses[-1].pose.orientation.y, 
                                      mesg.poses[-1].pose.orientation.z).yaw()
      cur_time = mesg.poses[-1].header.stamp.secs + \
                  mesg.poses[-1].header.stamp.nsecs*1e-9
      vio_yaw.append(cur_yaw)
      vio_time.append(cur_time)
      vio_full_traj = mesg.poses
   elif topic.split("/")[-1] == "cpu":
     cpu_consumption[topic.split("/")[2]].append(mesg.data)
   elif topic.split("/")[-1] == "mem":
     mem_consumption[topic.split("/")[2]].append(mesg.data)
     
#%%     
##### Post-process #####
# cut wifi data to vio length
cut_point = len(vio_position)

wifi_position = np.array(wifi_position)[:cut_point, :]
vio_position = np.array(vio_position)

wifi_yaw = np.array(wifi_yaw)[:cut_point]
vio_yaw = np.array(vio_yaw)

wifi_time = np.array(wifi_time)[:cut_point]
vio_time = np.array(vio_time) # !!! WARNING: This is currently zero due to bug in C++ package

final_ap_poses = np.array(ap_poses[-1][::2]) # num_aps x 3

vio_final_position = []; vio_final_yaw = []; vio_final_times = []
for pose in tqdm(vio_full_traj):
   vio_final_position.append([pose.pose.position.x, 
                              pose.pose.position.y, 
                              pose.pose.position.z])
   vio_final_yaw.append(gtsam.Rot3.Quaternion(pose.pose.orientation.w, 
                                              pose.pose.orientation.x, 
                                              pose.pose.orientation.y, 
                                              pose.pose.orientation.z).yaw())
   cur_time = pose.header.stamp.secs + pose.header.stamp.nsecs*1e-9
   vio_final_times.append(cur_time)

cut_point_final = len(vio_final_position)
   
wifi_final_position = []; wifi_final_yaw = []; wifi_final_times = []
for pose in wifi_full_traj:
   wifi_final_position.append([pose.pose.position.x, 
                              pose.pose.position.y, 
                              pose.pose.position.z])
   wifi_final_yaw.append(gtsam.Rot3.Quaternion(pose.pose.orientation.w, 
                                              pose.pose.orientation.x, 
                                              pose.pose.orientation.y, 
                                              pose.pose.orientation.z).yaw())
   cur_time = pose.header.stamp.secs + pose.header.stamp.nsecs*1e-9
   wifi_final_times.append(cur_time)

num_aps_over_time = list(map(len, ap_poses))
idx_ap_added = np.where(np.diff(num_aps_over_time)  != 0)[0]
# append 0 to incorporate that AP 0 was added at the initial point
idx_ap_added = np.hstack(([0], idx_ap_added))
times_ap_added = wifi_time[idx_ap_added]

#%%
dict_to_save = {"wifi_position": wifi_position, "vio_position": vio_position, 
                "wifi_yaw": wifi_yaw, "vio_yaw": vio_yaw, 
                "wifi_position_final": np.array(wifi_final_position)[:cut_point_final, :], 
                "vio_position_final": np.array(vio_final_position), 
                "wifi_yaw_final": np.array(wifi_final_yaw)[:cut_point_final],
                "vio_yaw_final": np.array(vio_final_yaw),
                "wifi_time_final": np.array(wifi_final_times)[:cut_point_final], 
                "wifi_time": wifi_time,
                "ap_poses": final_ap_poses, "memory": mem_consumption, 
                "cpu": cpu_consumption,
                "times_ap_added": times_ap_added}
sio.savemat(saved_mat_file_name, dict_to_save)
#%%
bag_in.close()
