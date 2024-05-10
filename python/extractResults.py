"""
This script will take input as a bag file, a csi-data mat file, AoA and AoD measurements 
and create a new bag file with this CSI data stored in the bag. The CSI data stored will be 
time-synced to the best extent possible. 
"""
computer = ""

import numpy as np
import rosbag
import scipy.io as sio
import sys
import os
if computer == "cuda":
   sys.path.append("/home/aditya/Research/p2slam/python/sim_proc")
   sys.path.append("/home/aditya/Research/utilities/python")
   sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
from tqdm import tqdm
import pdb
from sensor_msgs import point_cloud2
import gtsam
import h5py as hio
import matplotlib.pyplot as plt

sys.path.append('/home/aarun/Research/utilities/python/')
sys.path.append('/home/aarun/Research/p2slam/python/sim_proc/')
sys.path.append('/home/aarun/Research/p2slam/python/')

import scipy.optimize as opt
from graph_helpers import apply_global_rot, correct_bias


#%%
"""
/wifio/path_robot /wifio/pose_ap 
/cpu_monitor/wifio_test1/cpu /cpu_monitor/wifio_test1/mem 
/cpu_monitor/feature_tracker/cpu /cpu_monitor/feature_tracker/mem 
/vio/feature_cloud /vio/path_robot 
/cpu_monitor/vio_test1/cpu /cpu_monitor/vio_test1/mem
"""

   
bag_dir = "/home/aarun/Research/data/viofi/8-4/results_wifi_camera.bag"
dataset_dir = "/home/aarun/Research/data/p2slam_realworld/p2slam_atk/8-28-edge-aps-3"

#####################
#####################

input_bag_file = os.path.join(bag_dir)
times_file = os.path.join(dataset_dir, "times.mat")

opt_times = np.squeeze(sio.loadmat(times_file)["labels_time"])

bag_in = rosbag.Bag(input_bag_file, 'r')

ap_poses = []

wifi_position = [] # realtime traj
wifi_yaw = [] # realtime traj
wifi_time = []

vio_position = [] # realtime traj
vio_yaw= [] # realtime traj
vio_time = []

vio_pc = None
wifi_full_traj = None
vio_full_traj = None
   
cpu_consumption = {"feature_tracker":[], "vio_test1": [], "wifio_test1": []}
mem_consumption = {"feature_tracker":[], "vio_test1": [], "wifio_test1": []}
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
      
   elif topic == "/vio/feature_cloud":
      vio_pc = list(point_cloud2.read_points(mesg, skip_nans=True, 
                                             field_names = ("x", "y", "z")))
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
                "cpu": cpu_consumption, "pointcloud": np.array(vio_pc), 
                "times_ap_added": times_ap_added}
sio.savemat("final_results_8-4.mat", dict_to_save)
#%%
bag_in.close()

#%% Load synced data from matlab
synced_data = hio.File("/home/aarun/Research/data/viofi/8-4/all_labels_8-4.mat", 
                       'r')
synced_labels = {'labels_opt': None, 'labels_wifi_rt': None, 
                 'labels_wifi_final': None, 'labels_vio_rt': None, 
                 'labels_vio_final': None}

for ii, key in enumerate(synced_labels.keys()):
   synced_labels[key] = np.array(synced_data[synced_data['all_labels'][ii, 0]]).T

#%% Apply RATB to all labels_wifi_* and labels_vio_*
# apply global rotation to all the points to confirm the effects of relative
# rotation shift
rot_min = -180*np.pi/180
rot_max = 180*np.pi/180
corrected_labels = {'labels_opt': None, 'labels_wifi_rt': None, 
                    'labels_wifi_final': None, 'labels_vio_rt': None, 
                    'labels_vio_final': None}

cut_point = 0
pivot_point = 0
corrected_labels['labels_opt'] = synced_labels['labels_opt'][cut_point:]
# corrected_labels['labels_rtab'] = synced_labels['labels_rtab'][cut_point:]

for cur_label in ['labels_wifi_rt', 'labels_wifi_final', 'labels_vio_rt', 
                 'labels_vio_final']:
   rot_best, min_err, _, _ = opt.fminbound(apply_global_rot, rot_min, rot_max, \
                                           args=(synced_labels[cur_label][cut_point:],
                                                 pivot_point, 
                                                 synced_labels['labels_opt'][cut_point:]), \
                                           full_output=True)
   R_rotated = np.array([[np.cos(rot_best), -np.sin(rot_best)],
                                      [np.sin(rot_best), np.cos(rot_best)]])
   _, corrected_labels[cur_label] = apply_global_rot(rot_best, synced_labels[cur_label][cut_point:],
                                                     pivot_point, 
                                                     synced_labels['labels_opt'][cut_point:], 
                                                     ret_only_err=False)
# sio.savemat("/home/aarun/Research/data/viofi/8-28/corrected_labels_8-28.mat", 
#             corrected_labels)

#%%
import gtsam

# corrected_labels = {'labels_opt': None, 'labels_rtab': None, 'labels_wifi_rt': None, 
#                     'labels_wifi_final': None, 'labels_vio_rt': None, 
#                     'labels_vio_final': None}

corrected_labels['labels_opt'] = synced_labels['labels_opt'][cut_point:]
# corrected_labels['labels_rtab'] = synced_labels['labels_rtab'][cut_point:]

for cur_label in ['labels_rtab', 'labels_wifi_rt', 'labels_wifi_final', 'labels_vio_rt', 
                 'labels_vio_final']:
   temp_labels, R, t = \
         correct_bias(np.hstack((corrected_labels[cur_label][:, :2], np.zeros((len(corrected_labels[cur_label]), 1)))), 
                      np.hstack((synced_labels['labels_opt'][:, :2], np.zeros((len(synced_labels[cur_label]), 1)))))
         
   # temp_yaw = np.diff(temp_labels, axis=0)
   # temp_yaw = np.arctan2(temp_yaw[:, 1], temp_yaw[:, 0])
   # temp_yaw = np.hstack((temp_yaw, temp_yaw[-1]))
   
   yaw_correction = gtsam.Rot3(R).yaw()
   temp_yaw = synced_labels[cur_label][:, 2] + yaw_correction
   corrected_labels[cur_label] = np.hstack((temp_labels[:, :2], temp_yaw[:, None]))
#%%
plt.figure()
# plt.plot(synced_labels['labels_opt'][:, 0], 
#           synced_labels['labels_opt'][:, 1])
# plt.plot(synced_labels['labels_wifi_final'][:, 0], 
#           synced_labels['labels_wifi_final'][:, 1])
# plt.plot(synced_labels['labels_rtab'][:, 0], 
#           synced_labels['labels_rtab'][:, 1])
plt.plot(corrected_labels['labels_opt'][:, 0], 
          corrected_labels['labels_opt'][:, 1])
plt.plot(corrected_labels['labels_wifi_final'][:, 0], 
          corrected_labels['labels_wifi_final'][:, 1])
# plt.plot(corrected_labels['labels_rtab'][:, 0], 
#           corrected_labels['labels_rtab'][:, 1])

plt.figure();
plt.plot(corrected_labels['labels_wifi_final'][:, 1]); 
plt.plot(corrected_labels['labels_opt'][:, 1])

#%%

sio.savemat("/home/aarun/Research/data/viofi/8-4/corrected_labels_8-4.mat", 
            corrected_labels)