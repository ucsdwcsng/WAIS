"""
This script will take input as a bag file, a csi-data mat file, AoA and AoD measurements 
and create a new bag file with this CSI data stored in the bag. The CSI data stored will be 
time-synced to the best extent possible. 
"""
computer = "cuda"

import numpy as np
import rosbag
import rospy
import scipy.io as sio
import h5py as hio
import sys
import os
if computer == "cuda":
   sys.path.append("/home/aditya/Research/p2slam/python/sim_proc")
   sys.path.append("/home/aditya/Research/utilities/python")
   sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
from graph_helpers import load_angle_predictions
from wifi_backend.msg import Aoa
from tqdm import tqdm
import pdb

N_RX_ANT = 4
N_TX_ANT = 4
MAX_RSSI_NORMALIZED = -10

def createAoAMessage(time, ap_id, rssi, aoa, aoa_local=None):
   # See message definition in gtsam_backend/msg/Aoa.msg
   valToAdd = Aoa() 
   valToAdd.header.stamp = time
   valToAdd.ap_id = ap_id
   valToAdd.n_tx = len(aoa)
   valToAdd.n_rx = N_RX_ANT
   valToAdd.rssi = list(np.array(rssi, dtype=np.int32))
   valToAdd.aoa = list(aoa)
   if aoa_local is not None:
      valToAdd.aoa_local = list(aoa_local)
      
   return valToAdd

def combine_bag(bag_in, bag_out, csi_data, angle_data):
   """Note: Ignore storing CSI data for now, to keep bag files small"""
   print("Copying data to new Bag file")
   init_time = None
   for topic, mesg, t in tqdm(bag_in.read_messages()):
      if init_time is None:
         init_time = t.to_sec()
      bag_out.write(topic, mesg, t)
   
   print("Adding Bot Aoa data to new Bag file")
   for t, v, v_local, v_rssi in tqdm(zip(angle_data['times'], angle_data['bot_angle'], 
                                         angle_data['bot_angle_local'], angle_data['bot_rssi'])):
      
      # ignore all AoA packets before the first odom/camera frame
      if t < init_time:
         continue
      
      for ap_id, (v_ap, v_ap_local, v_ap_rssi) in enumerate(zip(v, v_local, v_rssi)):
         timeToAdd = rospy.Time.from_sec(t)
         valToAdd = createAoAMessage(timeToAdd, ap_id, v_ap_rssi, v_ap, v_ap_local)
         
         bag_out.write(f'/wifi/bot_angle/ap_{ap_id}', valToAdd, timeToAdd)
         
   print("Adding AP Aoa data to new Bag file")
   for t, v, v_rssi in tqdm(zip(angle_data['times'], angle_data['ap_angle'],  
                                angle_data['ap_rssi'])):
      
      # ignore all AoA packets before the first odom/camera frame
      if t < init_time:
         continue
      
      for ap_id, (v_ap, v_ap_rssi) in enumerate(zip(v, v_rssi)):
         timeToAdd = rospy.Time.from_sec(t)
         valToAdd = createAoAMessage(timeToAdd, ap_id, v_ap_rssi, v_ap)      
         
         bag_out.write(f'/wifi/ap_angle/ap_{ap_id}', valToAdd, timeToAdd)
   
         
if __name__ == "__main__":

   
   dataset_dir = "/media/ehdd_8t1/aarun/Research/data/p2slam_realworld/p2slam_atk/11-22-atkinson-1st"
   angles_dir = "/media/ehdd_8t1/aarun/Research/data/p2slam_realworld/results/11-22-atkinson-1st"

   bag_name = "data_short.bag"
   cartographer_folder = "2021-11-22-16-12-04"
   algorithm = "svd_fft"
   all_odom_post_fix =  ""
   
   store_gnd = False
   
   #####################
   #####################
   
   dataset_file = os.path.join(dataset_dir, all_odom_post_fix, "channels_atk.mat")
   input_bag_file = os.path.join(dataset_dir, "cartographer_data", cartographer_folder, 
                                 bag_name)
   out_bag_name = bag_name.split('.')[0] + "_w_aoa_gnd.bag" if store_gnd \
                     else bag_name.split('.')[0] + f"_w_aoa_{algorithm}.bag"
   output_bag_file = os.path.join(dataset_dir, "cartographer_data", cartographer_folder, 
                                 out_bag_name)
   csi_file = "" # ignore for now
   times_file = os.path.join(dataset_dir, all_odom_post_fix, "times.mat")
   
   if len(csi_file) == 0:
      csi_data = None
   else:
      csi_data = sio.loadmat(csi_file)
   
   angle_times = np.squeeze(sio.loadmat(times_file)["labels_time"])

   print("Loading bag files")   
   bag_in = rosbag.Bag(input_bag_file, 'r')
   bag_out = rosbag.Bag(output_bag_file, 'w')
   
   aoa_pred_rad, aoa_gnd_rad, _, _, \
   aod_pred_rad, aod_gnd_rad, _, _, \
   aod_pred_rad_local, aod_gnd_rad_local, \
   aoa_hampel_rad, aod_hampel_rad, aod_hampel_rad_local, \
   quality = load_angle_predictions(angles_dir, algorithm)
   
   # data is stored in v7.3 mat, so need to use h5py
   data = hio.File(dataset_file, mode='r+')
   
   # Load RSSI values
   rssi_dict = {'ap': np.median(np.array(data['ap_rssi_synced']).T, axis=1),
                'cli': np.median(np.array(data['cli_rssi_synced']).T, axis=1)} # num_meas x num_aps
   
   num_aps = rssi_dict['ap'].shape[1]
   for ii in range(num_aps):
      max_rssi = np.max(rssi_dict['cli'][rssi_dict['cli'][:, ii]!=0, ii])
      push_up = MAX_RSSI_NORMALIZED - max_rssi 
      rssi_dict['cli'][rssi_dict['cli'][:, ii]!=0, ii] += push_up

   for ii in range(num_aps):
      max_rssi = np.max(rssi_dict['ap'][rssi_dict['ap'][:, ii]!=0, ii])
      push_up = MAX_RSSI_NORMALIZED - max_rssi 
      rssi_dict['ap'][rssi_dict['ap'][:, ii]!=0, ii] += push_up

   if len(rssi_dict['ap'].shape) == 2:
      rssi_dict['ap'] = rssi_dict['ap'][:,np.newaxis] # num_meas x num_rx_ant x num_aps
   if len(rssi_dict['cli'].shape) == 2:
      rssi_dict['cli'] = rssi_dict['cli'][:,np.newaxis] # num_meas x num_rx_ant x num_aps
   
   
   angle_data = {}
   
   if store_gnd:
      angle_data['ap_angle'] = aoa_gnd_rad
      angle_data['bot_angle'] = aod_gnd_rad # in the AP coordinates on the bot, between -pi and pi
      angle_data['bot_angle_local'] = aod_gnd_rad_local # in the local bot coordinates
   else:
      angle_data['ap_angle'] = aoa_pred_rad
      angle_data['bot_angle'] = aod_pred_rad # in the AP coordinates on the bot, between -pi and pi
      angle_data['bot_angle_local'] = aod_pred_rad_local # in the local bot coordinates
   
   angle_data['times'] = angle_times
   angle_data['bot_rssi'] = np.swapaxes(rssi_dict['ap'], 1, 2) # definition is swtiched; legacy p2slam AP is Bot and Cli is AP
   angle_data['ap_rssi'] = np.swapaxes(rssi_dict['cli'], 1, 2)
   
   print(f"Number of measurements: {len(angle_times)}")
   
   combine_bag(bag_in, bag_out, csi_data, angle_data)
   
   bag_in.close()
   bag_out.close()
#%%

