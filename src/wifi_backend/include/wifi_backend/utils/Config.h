#pragma once

#define NUM_APS 3 // Mostly to avoid dynamic allocation of subscribers
#define PI 3.141

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <unordered_set>

class Config {
public:
  /// Default constructor
  Config(){}

  std::string fixedId;

  /// Assumed "true" global gravity
  Eigen::Matrix<double, 3, 1, Eigen::DontAlign> gravity;

  /// Position of AP's in global frame
  Eigen::Matrix<double,3, NUM_APS, Eigen::DontAlign> ap_pos;
  Eigen::Matrix<double,3,1, Eigen::DontAlign> ap_pos_prior; // Unused if ap_loc_qual == 2
  int ap_loc_qual; /// AP location quality, 0: ground truth, 1: erroneous, 2: unknown

  /// RSSI filter
  Eigen::Matrix<double,NUM_APS,1, Eigen::DontAlign> rssi_thresh;
  double ap_addition_thresh;

  /// Angle magnitude filter
  Eigen::Matrix<double,2,1, Eigen::DontAlign> angle_thresh;

  /// 360 Angle Mag filter
  Eigen::Matrix<double,2,1, Eigen::DontAlign> angle_thresh_square;

  /// AP's to consider
  std::unordered_set<int> aps_to_include;

  /// Toggle use 360 angle thresholds
  bool use_square_thresh;

  /// Extra iterations of optimization
  int opt_extra_iterations;

  /// Noise values for Initialization
  double sigma_prior_rotation; // 0.1 rad on roll, pitch, yaw
  double sigma_prior_translation; // 30cm std on x, y, z
  double sigma_velocity; // 0.1 m/s
  double sigma_bias; // 0.1 m/s

  /// Noise values for Odometry
  double sigma_xy;
  double sigma_theta;

  double sigma_angle_bot; // launch file -- in degrees, after loading, in radians
  double sigma_angle_AP; // same as above

  /// Camera to odom transform; this was especially needed for Kimera
  Eigen::Matrix<double, 4, 4, Eigen::DontAlign> cam_to_odom;

    /// Set sensors to be used
  bool use_odom;
  bool use_ap_side_angles;
  bool use_bot_side_angles;


  /// Optimizer options
  double optimizer_lag;
  bool use_isam; // if True, use ISAM2 solver
  bool use_batch; // If True, uses BatchFixedLagSmoother
  double huber_param; // Huber function's internal parameter
  // NOTE: If both use_isam and use_batch are false, then uses
  // IncrementalFixedLagSmoother
  double relinearizeThreshold;
  int relinearizeSkip;
  bool cacheLinearizedFactors;
  bool enablePartialRelinearizationCheck;
  bool findUnusedFactorSlots;

  /// File storage options
  std::string data_dir; // data directory where all files are stored
  std::string graph_filename; // Filename to save the graph, in case of failure
  std::string features_filename; // Filename to store odom features
  bool store_features; // if True, stores odom and camera features
  std::ofstream features_stream; // filestream for storing features

  /// nexmon-specific topics
  std::string aoa_topic;

  /// Output path topic name post-fix
  std::string topic_post_fix;

  /// The minimum difference between a previously updated values and current value.
  double min_update_distance;

};
