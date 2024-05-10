#!/usr/bin/env python
import sys
import rospy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
# from gtsam import Rot3
# import quaternion
from scipy.spatial.transform import Rotation as R
import gtsam
import pdb

"""
Node to transform data from /odom topic into relative odometery
"""
class RelativeOdometry(object):
   def __init__(self, name_topic_in, name_topic_out):

      rospy.loginfo("Converting odometry information from " + name_topic_in +
                 " to relative odomtery, output topic: " + name_topic_out)

      self.cam_to_odom = gtsam.Pose3(gtsam.Rot3(gtsam.Point3(np.array([1, 0, 0])),
                                                gtsam.Point3(np.array([0, 1, 0])),
                                                gtsam.Point3(np.array([0, 0, 1]))).inverse(),
                                     gtsam.Point3(0, 0, 0))

      # self.cam_to_odom = gtsam.Pose3(gtsam.Rot3(gtsam.Point3(np.array([0,  0, 1])),
      #                                           gtsam.Point3(np.array([-1, 0, 0])),
      #                                           gtsam.Point3(np.array([0, -1, 0]))).inverse(),
      #                                gtsam.Point3(0, 0, 0))
      # self.cam_to_odom = gtsam.Pose3()
      rospy.loginfo(f"Camera to odom transform {self.cam_to_odom}")

      self.pub = rospy.Publisher(name_topic_out, PoseStamped, queue_size=5)
      self.sub = rospy.Subscriber(name_topic_in, Odometry, self.relative_odom,
                                  queue_size=5)

      self.prev_position = np.array([0, 0, 0])
      self.prev_theta = 0
      self.prev_rotmat = np.eye(3)
      self.prev_pose = gtsam.Pose3()

   def get_position_quat(self, msg):
      position = np.array([msg.pose.pose.position.x,
                           msg.pose.pose.position.y,
                           msg.pose.pose.position.z])

      gtsam_rot3 = gtsam.Rot3.Quaternion(msg.pose.pose.orientation.w,
                                   msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z)
      quat_rot_mat = gtsam_rot3.matrix()
      quat_yaw = gtsam_rot3.yaw()

      # quat = R.from_quat([msg.pose.pose.orientation.x,
      #                     msg.pose.pose.orientation.y,
      #                     msg.pose.pose.orientation.z,
      #                     msg.pose.pose.orientation.w])
      # quat_rot_mat = quat.as_matrix()
      
      # quat_yaw = quat.as_euler('zyx')[0]
      # pdb.set_trace()

      return position, quat_yaw, quat_rot_mat


   def relative_odom(self, cur_msg):
      # Convert IMG_MSG, of type Nav_msgs/Odometry to relative odometry of type
      # Geometry_msgs/PoseStamped
      cur_position, cur_theta, cur_rotmat = self.get_position_quat(cur_msg)
      cur_pose = gtsam.Pose3(gtsam.Rot3(cur_rotmat), gtsam.Point3(cur_position))
      cur_pose = cur_pose.compose(self.cam_to_odom.inverse())

      rel_pose = self.prev_pose.transformPoseTo(cur_pose)

      rel_position = rel_pose.translation()

      rel_quat = rel_pose.rotation().toQuaternion().coeffs()

      msg_pose = Pose(position=Point(rel_position[0],
                                     rel_position[1],
                                     rel_position[2]),
                      orientation=Quaternion(rel_quat[0], # x, y, z, w
                                             rel_quat[1],
                                             rel_quat[2],
                                             rel_quat[3]))

      # Put data into the message
      cur_relative_odom_msg = PoseStamped(pose=msg_pose)
      # Add the message header
      cur_relative_odom_msg.header.frame_id = "relOdom"
      cur_relative_odom_msg.header.stamp = cur_msg.header.stamp
      cur_relative_odom_msg.header.seq = cur_msg.header.seq

      self.prev_position = cur_position
      self.prev_theta = cur_theta
      self.prev_rotmat = cur_rotmat
      self.prev_pose = cur_pose
      # Publish the message
      self.pub.publish(cur_relative_odom_msg)



if __name__ == '__main__':
   argv = rospy.myargv(sys.argv)
   if len(argv) != 3:
      print("Usage:")
      print(argv[0] + " name_topic_in name_topic_out")
      print("Converts a RGB/BGR name_topic_in ImageImport topic into a Grayscale name_topic_out ImageImport topic")
      exit(0)
   rospy.init_node("odom_to_relOdom", anonymous=True)

   gs = RelativeOdometry(argv[1], argv[2])
   rospy.spin()
