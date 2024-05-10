import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import h5py as hio
from gtsam import Rot3
import pdb

#%% Utility functions
def extractVioPathFromText(file):
   robot_points = []
   robot_rots = []
   with open(file, 'r') as f:
      while True:
         line = f.readline()
         if not line:
            print("Done reading")
            break

         if "position" in line:
            cur_pt = []
            for ii in range(3):
               line = f.readline()
               cur_pt.append(float(line.split(":")[1]))
            robot_points.append(cur_pt)
            continue
         if "orientation" in line:
            cur_rot = []
            for ii in range(4):
               line = f.readline()
               cur_rot.append(float(line.split(":")[1]))
            robot_rots.append(cur_rot) # x, y, z, w
            continue
   # bring robot_points to coorindate frame of odom/gnd, and change rots to w, x, y, z format
   robot_points = np.array(robot_points)[:, [1, 0, 2]]
   robot_points[:, 1] *= -1
   return np.array(robot_points), np.array(robot_rots)[:, [3, 0, 1, 2]]


def extractPathFromMat(file, label="labels_imu"):
   """
   Extract robot poses from mat files,
   label == "labels_imu" for encoder+gyro (defualt)
         == "label_noise" for encoder only
         == "labels" for gnd truth poses from cartographer
   """
   data = hio.File(file, mode='r+')
   pos_bot = np.array(data[label]).T # x, y, yaw
   robot_points = np.zeros((len(pos_bot), 3))
   robot_points[:, :2] = pos_bot[:, :2]
   robot_rots = np.array([Rot3.Yaw(pos_bot[ii, 2]).quaternion()
                 for ii in range(len(pos_bot))]) # w, x, y, z
   return robot_points, robot_rots

def plotPath(robot_points, robot_rots, label, ax=None):
   if ax is None:
      plt.figure()
      ax = plt.axes(projection='3d')

   ax.plot(robot_points[:, 0], robot_points[:, 1], robot_points[:, 2],
           label=label)
   # ax.quiver(robot_points[:, 0], robot_points[:, 1], robot_points[:, 2],
   #           #todo)

   return ax

def axisEqual3D(ax):
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)


#%%
vioPathFile = "/media/ehdd_8t1/aarun/Research/data/viofi/traj_gnd.txt"
matFile = "/media/ehdd_8t1/aarun/Research/data/p2slam_realworld/p2slam_atk/11-22-atkinson-1st/channels_atk.mat"

vioPoints, vioRots = extractVioPathFromText(vioPathFile)
# odomPoints, odomRots = extractPathFromMat(matFile)
gndPoints, gndRots = extractPathFromMat(matFile, label='labels')

ax = plotPath(vioPoints, vioRots, "VIO")
# plotPath(odomPoints, odomRots, "Odom", ax=ax)
plotPath(gndPoints, gndRots, "Gnd", ax=ax)
axisEqual3D(ax)
ax.legend()
pdb.set_trace()


