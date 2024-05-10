#pragma once

#include <mutex>
#include <thread>
#include <deque>
#include <unordered_map>
#include <cmath>
#include <fstream>
#include <unordered_set>
#include <csignal>
#include <ros/ros.h>


// Graphs
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <boost/circular_buffer.hpp>

// Factors
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingFactor.h>

#include "utils/Config.h"
#include "utils/State.h"

#define CIRCULAR_BUFFER_SIZE 5  // decides time window over which stability of optimization is observed
#define STABILITY_THRESHOLD 20  // quantifies the stability criteria, if more than STABILITY_THRESHOLD measurments are updated,
                                // then unstable

using gtsam::symbol_shorthand::X; // Robot Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::A; // AP Pose3 (x,y,r, p, y)

typedef std::vector<std::pair<double, gtsam::State>> Trajectory;
typedef gtsam::BearingFactor<gtsam::Pose3, gtsam::Pose3> BearingFactor3D;
class GraphSolver {
public:
  explicit GraphSolver(Config* config) {
    
    // Set up config;
    // NOTE: This config will have new properties that would 
    // need to be incorporated
    this->config = config;
    this->graph = new gtsam::NonlinearFactorGraph();
    this->graph_wifi = new gtsam::NonlinearFactorGraph();
    this->cam_to_odom = gtsam::Pose3(config->cam_to_odom);

    gtsam::ISAM2Params isam_params;
    if (this->config->use_isam) {
      // ISAM2 solver
      isam_params.relinearizeThreshold = config->relinearizeThreshold;
      isam_params.relinearizeSkip = config->relinearizeSkip;
      isam_params.cacheLinearizedFactors = config->cacheLinearizedFactors;
      isam_params.enableDetailedResults = false; // ISAM2Result.detail is computed; useful for debugging only
      isam_params.enablePartialRelinearizationCheck = config->enablePartialRelinearizationCheck;
//      WARNING: This fails currently; incorrectly assumes variable key is missing even though it's present in Values
//      when used with camera factors?
      // isam_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
      gtsam::ISAM2GaussNewtonParams opt_params = boost::get<gtsam::ISAM2GaussNewtonParams>(isam_params.getOptimizationParams());
      isam_params.setFactorization("CHOLESKY"); // QR
      isam_params.print();
      this->isam2_wifi = new gtsam::ISAM2(isam_params);
    } else if (this->config->use_batch) {
      this->bfls = new gtsam::BatchFixedLagSmoother(this->config->optimizer_lag);
    } else {
      isam_params.relinearizeSkip = config->relinearizeSkip;
      isam_params.relinearizeThreshold = config->relinearizeThreshold;
      isam_params.findUnusedFactorSlots = true;
      // Following Kimera params
      isam_params.cacheLinearizedFactors = config->cacheLinearizedFactors;
      isam_params.findUnusedFactorSlots = config->findUnusedFactorSlots;
      isam_params.evaluateNonlinearError =  false;
      isam_params.enableDetailedResults = false;
      isam_params.enablePartialRelinearizationCheck = config->enablePartialRelinearizationCheck;
      isam_params.setFactorization("CHOLESKY");

      gtsam::ISAM2GaussNewtonParams opt_params = boost::get<gtsam::ISAM2GaussNewtonParams>(isam_params.getOptimizationParams());
      opt_params.wildfireThreshold = 0.001;
      isam_params.optimizationParams = opt_params;

      this->ifls = new gtsam::IncrementalFixedLagSmoother(this->config->optimizer_lag, isam_params);
    }
  }

  /// Will return true if the system is initialized
  inline bool is_initialized() const { return systeminitalized; }

    /// Function that takes in Relative Odometry measurments for use in Graph
  void addmeasurement_odom(double timestamp, Eigen::Vector3d relPosition, Eigen::Vector4d relQuat);

  /// Function that takes in AoA measurement at Robot or the AP and adds it to the graph
  void addmeasurement_aoa(double timestamp, size_t apId, double aoa, double rssi, bool botSided);

  /// This function will optimize the graph
  void optimize(double timestamp);

  ///  If using Batch optimizers, only update the values which were optimized.
  void updateRecentValuesIncremental();
  void updateRecentValuesBatch();

  /// Convert given AoA in 2D-XY Plane to a Unit3; z = 0
  static gtsam::Unit3 convertFromAoA(double aoa);

        /// This function returns the current state, return origin if we have not initialized yet
  gtsam::State get_state(size_t ct) {
    // Ensure valid states
    assert (ct <= ct_state);
    if (!values_all.exists(X(ct)))
      return gtsam::State();

  if (config->use_odom) {
      return values_optimized_wifi.exists(X(ct)) ?
             gtsam::State(values_optimized_wifi.at<gtsam::Pose3>(X(ct)),
                          ct)
             : gtsam::State(values_all.at<gtsam::Pose3>(X(ct)),
                            ct);
    }
  }

  gtsam::State get_state(const gtsam::Pose3& val, double ct) {
    return gtsam::State(val, ct);
  }

  gtsam::Pose3 get_ap_pose(size_t ap_id, bool &valid_pose) {
    // Request access to the optimized measurements
    std::unique_lock<std::mutex> opt_lock(opt_mutex);

    if (values_optimized_wifi.exists(A(ap_id))) {
      valid_pose = true;
      return gtsam::Pose3(values_optimized_wifi.at<gtsam::Pose3>(A(ap_id)));
    } else if (values_all.exists(A(ap_id))) {
      valid_pose = true;
      return gtsam::Pose3(values_all.at<gtsam::Pose3>(A(ap_id)));
    } else {
      valid_pose = false;
      return gtsam::Pose3();
    }
  }
  
  gtsam::State get_current_state() { 
    return get_state(ct_state);
  }

  Trajectory get_trajectory() {
    // Request access to the optimized measurements
    std::unique_lock<std::mutex> opt_lock(opt_mutex);

    // Return if we do not have any nodes yet
    if (values_all.empty()) {
      ROS_WARN("No values have been added to graph");
      Trajectory traj;
      traj.push_back(std::make_pair(0.0, gtsam::State()));
      return traj;
    }
    Trajectory trajectory;
    // Else loop through the states and return them
    for (const size_t i : allStatesId) {
      double timestamp = timestamp_lookup[i];
      trajectory.push_back(std::make_pair(timestamp, get_state(i)));
    }
    return trajectory;
  }

  static bool is_stable(boost::circular_buffer<double> &buff) {
    // check if all elements in buff is less than stability threshold
    return std::all_of(buff.begin(), buff.end(), [](const double ii) {return ii < STABILITY_THRESHOLD;}) ;
  }

  static bool is_very_unstable(boost::circular_buffer<double> &buff) {
    // check if all elements in buff is less than stability threshold
    return std::all_of(buff.begin(), buff.end(), [](const double ii) { return ii > STABILITY_THRESHOLD; });
  }

  Trajectory get_diff_trajectory(bool &valid) {
    // Request access to the optimized measurements
    std::unique_lock<std::mutex> opt_lock(opt_mutex);

    // Return if we do not have any nodes yet
    if (values_all.empty()) {
      Trajectory traj;
      traj.push_back(std::make_pair(-1, gtsam::State()));
      valid = false;
      return traj;
    }
    Trajectory trajectory;
    int num_changed = 0;
    // Else loop through the states and return them
    for (unsigned long cur_state_id : allStatesId) {
      if (!values_optimized_prev.exists(X(cur_state_id))) {
        gtsam::State cur_state = get_state(cur_state_id);

        if (diffTraj_changed.exists(X(cur_state_id))) {
          diffTraj_changed.update(X(cur_state_id), cur_state.pose());
        } else {
          diffTraj_changed.insert(X(cur_state_id), cur_state.pose());
        }
        diffTraj_stateids.insert(cur_state_id);

      } else {
        gtsam::Pose3 prev_wifi = values_optimized_prev.at<gtsam::Pose3>(X(cur_state_id));
        gtsam::Pose3 cur_wifi  = values_optimized_wifi.at<gtsam::Pose3>(X(cur_state_id));
        gtsam::Point3 translation_diff = prev_wifi.translation() - cur_wifi.translation();
        double normed_diff = gtsam::norm3(translation_diff);
        // if a point has been updates in the wifi graph by more than 10 cm, update it in camera graph
        if (normed_diff > config->min_update_distance) {

          num_changed++;

          if (diffTraj_changed.exists(X(cur_state_id))) {
            diffTraj_changed.update(X(cur_state_id), cur_wifi);
          } else {
            diffTraj_changed.insert(X(cur_state_id), cur_wifi);
          }
          diffTraj_stateids.insert(cur_state_id);

        }
      }
    }
    if (is_stable(*num_changed_history)) {
      for (size_t ct : diffTraj_stateids) {
        double timestamp = timestamp_lookup[ct];
        trajectory.push_back(std::make_pair(timestamp, get_state(diffTraj_changed.at<gtsam::Pose3>(X(ct)), ct)));
      }
      diffTraj_stateids.clear();
      diffTraj_changed.clear();
    }
//    else {
//      ROS_INFO("Graph unstable, stashing prior changes for future use");
//    }
    num_changed_history->push_back(num_changed);
    valid = true;
    return trajectory;
  }
  
  /// Clear all new variables
  void clean() {
    // Request access to the optimized measurements
    std::unique_lock<std::mutex> opt_lock(opt_mutex);
    delete this->graph;
    delete this->graph_wifi;
    if (this->config->use_isam) {
      delete this->isam2_wifi;
    } else if (this->config->use_batch) {
      delete this->bfls;
    } else {
      delete this->ifls;
    }

    ct_state_lookup_wifi.clear();
    timestamp_lookup.clear();
    values_new_wifi.clear();
    values_all.clear();
    values_optimized_wifi.clear();
    std::cout<<"Cleaned all variables";
  }
private:
  /// Functions
  // Function which will try to initalize our graph using the current odom measurements
  void initialize(double timestamp);

  // Add odom factor based on differential pose3
  void add_odom_factor(double timestamp, const gtsam::Pose3& diffPose3);

  /// Members
  // Config object (has all sensor noise values)
  Config* config;

  // New factors that have not been optimized yet; wifi and odom factors only
  gtsam::NonlinearFactorGraph* graph_wifi;

  // Master non-linear GTSAM graph, all created factors
  gtsam::NonlinearFactorGraph* graph;

  // New nodes that have not been optimized; only those added by odom
  gtsam::Values values_new_wifi;

  // All created nodes
  gtsam::Values values_all;

  // Camera to odom transform
  gtsam::Pose3 cam_to_odom;

  // Optimized values
  gtsam::Values values_optimized_wifi;
  gtsam::Values values_optimized_prev;
  gtsam::Values cur_smoothed_vals;

  // New nodes added by WiFi measurements
  std::unordered_set<int> aps_seen;

  // ISAM2 solvers
  gtsam::ISAM2* isam2_wifi;

  // Batch solvers
  gtsam::BatchFixedLagSmoother* bfls;
  gtsam::IncrementalFixedLagSmoother* ifls;
//  gtsam::IncrementalFixedLagSmoother ifls_backup;

  gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;

  // Current ID of state and features
  size_t ct_state = 0;
  size_t ct_camera_factor_index = 0;

  // Boolean that tracks if we have initialized
  bool systeminitalized = false;

  std::unordered_map<double, size_t> ct_state_lookup_wifi; // ct state based on timestamp
  std::unordered_map<size_t, double> timestamp_lookup;
  std::unordered_map<size_t, double> timestamp_lookup_ap;


  /// Lookup tables for features
  std::unordered_map<int, std::uint64_t> prior_factor_index_lookup;

  // Data from odom measurements
  std::mutex odom_mutex;
  std::mutex opt_mutex;
  std::deque<double> odom_times;

  std::deque<size_t> allStatesId;

  gtsam::FactorIndices camera_factors_removed;

  boost::circular_buffer<double> *num_changed_history = new boost::circular_buffer<double>(CIRCULAR_BUFFER_SIZE);
  gtsam::Values diffTraj_changed;
  std::set<size_t> diffTraj_stateids;
};
