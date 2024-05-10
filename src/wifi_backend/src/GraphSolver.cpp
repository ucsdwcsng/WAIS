#include "GraphSolver.h"

void GraphSolver::add_odom_factor(double timestamp, const gtsam::Pose3& diffPose3) {
  // Create between factors for odometry measurement
  gtsam::noiseModel::Diagonal::shared_ptr odomNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<
                                         config->sigma_theta, config->sigma_theta, config->sigma_theta,
                                         config->sigma_xy, config->sigma_xy, config->sigma_xy).finished());

  gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr bw_factor(
          new gtsam::BetweenFactor<gtsam::Pose3>(X(ct_state), X(ct_state + 1), diffPose3, odomNoise));
  graph_wifi->push_back(bw_factor);
  graph->push_back(bw_factor);

  gtsam::Pose3 curPose = values_optimized_wifi.exists(X(ct_state)) ?
                 values_optimized_wifi.at<gtsam::Pose3>(X(ct_state))
                                                      : values_all.at<gtsam::Pose3>(X(ct_state));
  // From this we should predict where we will be at the next time (t=K+1)
  gtsam::Pose3 newPose = curPose.compose(diffPose3);
  ct_state++;

  values_new_wifi.insert(X(ct_state), newPose);
  values_all.insert(X(ct_state), newPose);

  newTimestamps[X(ct_state)] = timestamp;
  timestamp_lookup[ct_state] = timestamp;
  allStatesId.push_back(ct_state);
}

void GraphSolver::addmeasurement_odom(double timestamp, Eigen::Vector3d relPosition, Eigen::Vector4d relQuat) {
  if (config->use_odom) {
    // We should try to initialize now
    // Or add the current a new Odom measurement and state!
    if(!systeminitalized) {
      initialize(timestamp);
      // Return if we have not initialized the system yet
      if (!systeminitalized)
        return;
    }
    // Request access to the odom measurements
    std::unique_lock<std::mutex> odom_lock(odom_mutex);

    // Request access to the optimized measurements
    std::unique_lock<std::mutex> opt_lock(opt_mutex);

    odom_times.push_back(timestamp);
    gtsam::Pose3 diffPose3 = gtsam::Pose3(gtsam::Rot3::Quaternion(relQuat[0], relQuat[1], relQuat[2],relQuat[3]),
                                  gtsam::Point3(relPosition[0], relPosition[1], relPosition[2]));

    // Apply the cam to odom transform if present; usually should be identity
//    diffPose3 = this->cam_to_odom.compose(diffPose3);

    add_odom_factor(timestamp, diffPose3);
  }
}

gtsam::Unit3 GraphSolver::convertFromAoA(double aoa) {
  // In 3D pose, need to convert the 1D AoA to 3D Unit vector,
  // parameterized by both azimuth and polar angle
  return gtsam::Unit3(cos(aoa), sin(aoa), 0);
}

void GraphSolver::addmeasurement_aoa(double timestamp, size_t apId, double aoa, double rssi, bool botSided) {

  // Return if the node already exists in the graph
//  if (ct_state_lookup_wifi.find(timestamp) != ct_state_lookup_wifi.end()) {
//    ROS_INFO("Timestamp already present in graph, returning");
//    return;
//  }

  if(!systeminitalized) {
    initialize(timestamp);
    // Return if we have not initialized the system yet
    if (!systeminitalized)
      ROS_INFO("Unable to intialize system, returning");
    return;
  }

  // The RSSI value is too low for addition and the AP is not seen yet
  if (rssi < config->ap_addition_thresh && aps_seen.find(apId) == aps_seen.end()) {
//    ROS_INFO("Rejecting, AP %zu, not seen yet. %f < %f rssi", apId, rssi, config->ap_addition_thresh);
    return;
  }
  // Add bearing factor
  const gtsam::noiseModel::mEstimator::Huber::shared_ptr noiseBase(
            new gtsam::noiseModel::mEstimator::Huber(config->huber_param));

  if (botSided) {
    const gtsam::noiseModel::Isotropic::shared_ptr noiseVar =
            gtsam::noiseModel::Isotropic::Sigma(2, config->sigma_angle_bot);
    gtsam::noiseModel::Robust::shared_ptr aoaNoise = gtsam::noiseModel::Robust::Create(noiseBase, noiseVar);
    BearingFactor3D::shared_ptr cur_factor(new BearingFactor3D(X(ct_state), A(apId),
                                                                  convertFromAoA(aoa),
                                                                  aoaNoise));
    graph_wifi->push_back(cur_factor);
    graph->push_back(cur_factor);
  } else {
    const gtsam::noiseModel::Isotropic::shared_ptr noiseVar =
            gtsam::noiseModel::Isotropic::Sigma(2, config->sigma_angle_AP);
    gtsam::noiseModel::Robust::shared_ptr aoaNoise = gtsam::noiseModel::Robust::Create(noiseBase, noiseVar);
    BearingFactor3D::shared_ptr cur_factor(new BearingFactor3D(A(apId), X(ct_state),
                                                                  convertFromAoA(aoa),
                                                                  aoaNoise));

    graph_wifi->push_back(cur_factor);
    graph->push_back(cur_factor);
  }
  gtsam::KeyVector cur_keys;
  std::unordered_set<size_t> cur_aps_in_graph;
  if (!config->use_isam && !config->use_batch) {
    bool no_aps = true;
    cur_keys = ifls->getFactors().keyVector();
    for (auto k : cur_keys) {
      if (gtsam::symbolChr(k) == 'a') {
        no_aps = false;
//        std::cout << gtsam::symbolChr(k) << gtsam::symbolIndex(k) << "; ";
        cur_aps_in_graph.insert(gtsam::symbolIndex(k));
      }
    }
    if (no_aps) {
      std::cout << "No AP's in graph currently\n";
    }
  }
  // Add AP positions and priors, usually the case the first time an AP is seen, when the RSSI threshold is small enough
  if ( aps_seen.find(apId) == aps_seen.end() ||
                ( !config->use_isam &&
                  cur_aps_in_graph.find(apId) == cur_aps_in_graph.end() &&
                  !values_new_wifi.exists(A(apId)) &&
                  (timestamp - timestamp_lookup_ap[apId]) >= config->optimizer_lag)
     ) {
    //  if (aps_seen.find(apId) == aps_seen.end()) {
    // insert AP initial values
    // (timestamp - timestamp_lookup_ap[apId]) >= config->optimizer_lag
    // std::find(cur_keys.begin(), cur_keys.end(), A(apId)) == cur_keys.end())
    gtsam::Pose3 cur_ap_pose;
    if (config->ap_loc_qual != 2) {
      cur_ap_pose = gtsam::Pose3(gtsam::Rot3::Yaw(config->ap_pos(2, apId)),
                                 gtsam::Point3(config->ap_pos(0, apId),
                                               config->ap_pos(1, apId),
                                               0));
      ROS_INFO("[ADD MEAS] Initialized new AP %zu at x, y, theta = (%.1f, %0.1f, %0.1f) at time %f",
               apId, cur_ap_pose.x(), cur_ap_pose.y(), cur_ap_pose.rotation().yaw(), timestamp);

    } else if (!config->use_isam && values_optimized_wifi.exists(A(apId))) {
      cur_ap_pose = values_optimized_wifi.at<gtsam::Pose3>(A(apId));
      ROS_INFO("[ADD MEAS] Re-adding AP pose %zu at x, y, theta = (%.1f, %0.1f, %0.1f) at time %f",
               apId, cur_ap_pose.x(), cur_ap_pose.y(), cur_ap_pose.rotation().yaw(), timestamp);

    } else {
      // Initialize close to the given robot position
      gtsam::Pose3 cur_robot_pose = values_all.at<gtsam::Pose3>(X(ct_state));
      gtsam::Pose3 delta_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, -1, 0));
      cur_ap_pose = delta_pose.compose(gtsam::Pose3(gtsam::Rot3(), cur_robot_pose.translation()));
      ROS_INFO("[ADD MEAS] Initialized new AP %zu at x, y, theta = (%.1f, %0.1f, %0.1f) "
               "near robot position at (%.1f, %0.1f) at time %f",
               apId, cur_ap_pose.x(), cur_ap_pose.y(), cur_ap_pose.rotation().yaw(),
               cur_robot_pose.x(), cur_robot_pose.y(), timestamp);
    }
    // Insert values only for the first time
    values_new_wifi.insert(A(apId), cur_ap_pose);
    newTimestamps[A(apId)] = timestamp;
    if (aps_seen.find(apId) == aps_seen.end())
      values_all.insert(A(apId), cur_ap_pose);

    // Setup AP position prior for the current AP
    // NOTE: If this is not set, then the graph becomes indeterminant
    gtsam::noiseModel::Diagonal::shared_ptr ap_noise = gtsam::noiseModel::Diagonal::Sigmas(
                                  (gtsam::Vector(6) << 0.01*PI/180, // rotation around x axis
                                                          0.01*PI/180, // rotation around y axis
                                                          config->ap_pos_prior(2), // rotation around z axis
                                                          config->ap_pos_prior(0),
                                                          config->ap_pos_prior(1),
                                                          0.01).finished());

    graph_wifi->add(gtsam::PriorFactor<gtsam::Pose3>(A(apId), cur_ap_pose, ap_noise));
    graph->add(gtsam::PriorFactor<gtsam::Pose3>(A(apId), cur_ap_pose, ap_noise));

    aps_seen.insert(apId);
    timestamp_lookup_ap[apId] = timestamp;
  }

  // Add ct state to map
  ct_state_lookup_wifi[timestamp] = ct_state;
}

// TODO: What about AP's and other landmarks here?
void GraphSolver::updateRecentValuesBatch() {
  for (unsigned long cur_state_id : allStatesId) {
    if (values_optimized_wifi.exists(X(cur_state_id))) {
      values_optimized_wifi.update(X(cur_state_id), bfls->calculateEstimate<gtsam::Pose3>(X(cur_state_id)));
    } else {
      values_optimized_wifi.insert(X(cur_state_id), bfls->calculateEstimate<gtsam::Pose3>(X(cur_state_id)));
    }
  }
}

// TODO: What about AP's and other landmarks here?
void GraphSolver::updateRecentValuesIncremental() {
  // Update the poses which are currently in view
  cur_smoothed_vals = ifls->calculateEstimate();
  for (gtsam::Key cur_key : cur_smoothed_vals.keys()) {
    auto cur_state_id = gtsam::symbolIndex(cur_key);
    auto cur_state_symbol = gtsam::symbolChr(cur_key);
    if (cur_state_symbol == 'x') {
      if (values_optimized_wifi.exists(X(cur_state_id))) {
        values_optimized_wifi.update(X(cur_state_id), ifls->calculateEstimate<gtsam::Pose3>(X(cur_state_id)));
      } else {
        values_optimized_wifi.insert(X(cur_state_id), ifls->calculateEstimate<gtsam::Pose3>(X(cur_state_id)));
      }
    } else if (cur_state_symbol == 'a'){
      if (values_optimized_wifi.exists(A(cur_state_id))) {
        values_optimized_wifi.update(A(cur_state_id), ifls->calculateEstimate<gtsam::Pose3>(A(cur_state_id)));
      } else {
        values_optimized_wifi.insert(A(cur_state_id), ifls->calculateEstimate<gtsam::Pose3>(A(cur_state_id)));
      }
    } else {
      ROS_WARN("Requested key update is neither AP nor Robot pose, it's %c", cur_state_symbol);
    }
  }
}

void GraphSolver::optimize(double timestamp) {

  // Request access to the optimized measurements
  std::unique_lock<std::mutex> opt_lock(opt_mutex);
  // Return if not initialized
  if (!systeminitalized && ct_state < 2) {
    ROS_INFO("No init");
    return;
  }
//  ROS_INFO("Optimizing");
  try {
    values_optimized_prev = gtsam::Values(values_optimized_wifi);
    if (config->use_isam) {
      // Add the odom factors
      isam2_wifi->update(*graph_wifi, values_new_wifi);
//      isam2_wifi->getVariableIndex();
      if (config->opt_extra_iterations != 0 && is_very_unstable(*num_changed_history)) {
        ROS_INFO("Graph very unstable, re-optimizing %d times", config->opt_extra_iterations);
        for (int it = 0; it < config->opt_extra_iterations; ++it) {
          isam2_wifi->update();
        }
      }
      values_optimized_wifi = isam2_wifi->calculateEstimate();
    } else {
      if (config->use_batch) {
        bfls->update(*graph_wifi, values_new_wifi, newTimestamps);
        updateRecentValuesBatch();
      } else {
//        ROS_INFO("================================");
//        ifls->print();
//        graph_wifi->print();
//        values_new_wifi.print();
//        for(auto newTimestamp : newTimestamps){
//          std::cout << newTimestamp.first << " " << newTimestamp.second << "\n";
//        }
//        ROS_INFO("================================");

        ifls->update(*graph_wifi, values_new_wifi, newTimestamps);
        updateRecentValuesIncremental();
      }
    }
  } catch (gtsam::IndeterminantLinearSystemException &e) {
    ROS_ERROR("VIOFI: WiFi graph indeterminate linear system exception!");
    std::cerr << e.what() << std::endl;
//    std::ofstream os(config->graph_filename);
//    graph->saveGraph(os, values_all);
//    os.flush();
//    os.close();
    clean();
    exit(EXIT_FAILURE);
  } catch (std::invalid_argument &e) {
    ROS_ERROR("VIOFI: invalid argument exception!");
    std::cerr << e.what() << std::endl;
    std::cout << "Graph factors \n";
    ifls->getFactors().print();
    std::cout << "Current Values \n";
    ifls->calculateEstimate().print();
    clean();
    exit(EXIT_FAILURE);
  }
  // Remove the used up nodes
  newTimestamps.clear();
  // allStatesId.clear();

  // Remove the used up factors and values
  graph_wifi->resize(0);
  values_new_wifi.clear();

}

void GraphSolver::initialize(double timestamp) {
  // If we have already initialized, then just return
  if (systeminitalized)
    return;

  // Create prior factor and add it to the graph
  gtsam::State prior_state = gtsam::State(gtsam::Pose3());
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) <<
                gtsam::Vector3::Constant(config->sigma_prior_rotation),
                gtsam::Vector3::Constant(config->sigma_prior_translation)).finished());
  gtsam::noiseModel::Isotropic::shared_ptr v_noise = gtsam::noiseModel::Isotropic::Sigma(3, config->sigma_velocity);
  gtsam::noiseModel::Isotropic::shared_ptr b_noise = gtsam::noiseModel::Isotropic::Sigma(6, config->sigma_bias);

  graph_wifi->add(gtsam::PriorFactor<gtsam::Pose3>(X(ct_state), prior_state.pose(), pose_noise));
  graph->add(    gtsam::PriorFactor<gtsam::Pose3>(X(ct_state), prior_state.pose(), pose_noise));


  // Add initial state to the graph
  values_new_wifi.insert(    X(ct_state), prior_state.pose());
  values_all.insert(X(ct_state), prior_state.pose());
  newTimestamps[X(ct_state)] = timestamp;
  allStatesId.push_back(ct_state);

  timestamp_lookup[ct_state] = timestamp;

  // Add ct state to map
  ct_state_lookup_wifi[timestamp] = ct_state;
  timestamp_lookup[ct_state] = timestamp;


  if (config->use_odom) {
    systeminitalized = true;
  }

  // Debug info
  ROS_INFO("Initialized WiFi graph at Time %0.4f", timestamp);
  ROS_INFO("\033[0;32m[INIT]: System Initialized ? %d \033[0m", systeminitalized);

}
