/* 
 * Lu Gan
 * ganlu@umich.edu
 *
 * Aditya Arun
 * aarun@ucsd.edu
 */

/*
 * Notes:
 * Publishes: /wifio/pose_robot/, /wifio/path_robot/, /wifio/pose_ap/, /wifio/data_odom/
 * Subscribes: /wifio/data_odom, /wifi/ap_angle/ap_xx, /wifi/bot_angle/ap_xx, /wifio/data_cam
 * */

#include <vector>
#include <cassert>
#include <string>
#include <ctime>
#include <cmath>
// Ros related
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rf_msgs/Aoa.h>

#include "GraphSolver.h"
#include "utils/Config.h"
#include "utils/Convert.h"


// Macros
#define FEAT_ID 0
#define FEAT_X_VEL 3
#define FEAT_Y_VEL 4

// Functions
void setup_config(ros::NodeHandle& nh, Config* pConfig);
void setup_subpub(ros::NodeHandle& nh);
void print(std::unordered_set<int> const &s);

void handle_measurement_odom(const geometry_msgs::PoseStampedPtr& msg);
void handle_measurement_uv(const sensor_msgs::CompressedImage msg);
void handle_measurement_single_aoa(const rf_msgs::AoaPtr& msg);

void optimize_graph(double timestamp);
void publish_state(double timestamp, gtsam::State& state);
void publish_trajectory(double timestamp, Trajectory& trajectory, const ros::Publisher& pub);
void publish_apMarkers(double timestamp);

nav_msgs::Path get_trajectory(Trajectory& trajectory, bool& valid);

ros::Subscriber subRelOdomMeas;
ros::Subscriber subCamMeas;
ros::Subscriber subAoa;

ros::Publisher pubPoseRobot;
ros::Publisher pubPathRobot;
ros::Publisher pubDiffPath;
ros::Publisher pubPoseAP;

// Master config and graph object
Config* config;
GraphSolver* graphsolver;
int poses_seq = 0;
int skip = 0;

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "vio");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");

  config = new Config();
  setup_config(nhPrivate, config);
  setup_subpub(nh);
  
  sleep(2);
  
  graphsolver = new GraphSolver(config);

  ros::spin(); // Use rosnode kill to exit this function

  if (config->store_features) {
    ROS_INFO("Closing Features file");
    config->features_stream.flush();
    config->features_stream.close();
  }
  // Store final Wifi trajectory in bag file;
  bool trajValid = false;
  Trajectory wifi_traj = graphsolver->get_trajectory();
  nav_msgs::Path wifi_traj_msg = get_trajectory(wifi_traj, trajValid);

  if (trajValid) {
    rosbag::Bag trajBag;

    time_t now = time(0);
    std::string bagFileName = config->data_dir + "wifiTrajBag_";
    bagFileName += ctime(&now);

    trajBag.open(bagFileName, rosbag::BagMode::Write);

    trajBag.write("wifiFinalTraj", ros::Time::now(), wifi_traj_msg);

    trajBag.close();
    ROS_INFO("Done writing WiFi trajectory message to %s", bagFileName.c_str());

  } else {
    ROS_ERROR("Cannot write WiFi trajectory message to bag file");
  }

  graphsolver->clean();
  delete config;
  delete graphsolver;

  return EXIT_SUCCESS;
}

void setup_config(ros::NodeHandle& nh, Config* pConfig) {

  pConfig->fixedId = "global";
  nh.param<std::string>("fixedId", pConfig->fixedId, pConfig->fixedId);
  // Load global gravity
  std::vector<double> gravity = {0, 0, 9.8};
  nh.param<std::vector<double>>("gravity", gravity, gravity);
  for (size_t i = 0; i < 3; ++i) pConfig->gravity(i, 0) = gravity.at(i);


  // Activate certain sensors only
  nh.param<bool>("useOdom", pConfig->use_odom, true);
  nh.param<bool>("useAPAngles", pConfig->use_ap_side_angles, true);
  nh.param<bool>("useBotAngles", pConfig->use_bot_side_angles, true);

  // Read in AP positions and related information
  nh.param<int>("apLocQual", pConfig->ap_loc_qual, 2);

  nh.param<double>("minUpdateDistance", pConfig->min_update_distance, 0.1);

  std::vector<double> ap_pos(NUM_APS*3, 0);
  nh.param<std::vector<double>>("apPos", ap_pos, ap_pos);
  for (size_t i = 0; i < NUM_APS*3; ++i) pConfig->ap_pos(i) = ap_pos.at(i);

  std::vector<double> ap_pos_prior = {0, 0, 0};
  nh.param<std::vector<double>>("apPosPrior", ap_pos_prior, ap_pos_prior);
  for (size_t i = 0; i < 3; ++i) pConfig->ap_pos_prior(i) = ap_pos_prior.at(i);
  pConfig->ap_pos_prior(2) *= PI / 180;

  // Set AoA topic
  nh.param<std::string>("aoaTopic", pConfig->aoa_topic, "aoa");

  // Read RSSI threshold parameters
  std::vector<double> rssi_thresh(NUM_APS, -100);
  nh.param<std::vector<double>>("rssiFilter", rssi_thresh, rssi_thresh);
  for (size_t i = 0; i < NUM_APS; ++i) pConfig->rssi_thresh(i) = rssi_thresh.at(i);

  nh.param<double>("apAdditionThresh", pConfig->ap_addition_thresh, -25.0);
  ROS_INFO("AP Addition Thresh: %f", pConfig->ap_addition_thresh);

  // Choose which AP's to use
  std::vector<int> aps_to_include(NUM_APS, -1);
//  std:iota(aps_to_include.begin(), aps_to_include.end(), 0); // Default consider all NUM_APS AP's
  nh.param<std::vector<int>>("apsToInclude", aps_to_include, aps_to_include);
  for (size_t i = 0; i < aps_to_include.size(); ++i) pConfig->aps_to_include.insert(aps_to_include.at(i));

  // Read in the camera to odom transform; this should be identity if odom is coming wrt baselink
  std::vector<double> cam_to_odom(4*4, 0);
  nh.param<std::vector<double>>("camToOdom", cam_to_odom, cam_to_odom);
  for (size_t i = 0; i < 4*4; ++i) pConfig->cam_to_odom(i) = (cam_to_odom.at(i));
  pConfig->cam_to_odom.transposeInPlace();

  // Read Angle threshold parameters, in radians
  std::vector<double> angle_thresh = {-90*PI/180, 90*PI/180};
  nh.param<std::vector<double>>("angleThresh", angle_thresh, angle_thresh);
  for (size_t i = 0; i < 2; ++i) pConfig->angle_thresh(i) = angle_thresh.at(i) * PI / 180;

  // Read Angle threshold parameters for the square array
  std::vector<double> angle_thresh_square = {20*PI/180, 160*PI/180};
  nh.param<std::vector<double>>("angleThreshSquare", angle_thresh_square, angle_thresh_square);
  for (size_t i = 0; i < 2; ++i) pConfig->angle_thresh_square(i) = angle_thresh_square.at(i) * PI / 180;

  // Decide which thresholds to use
  bool use_square_thresh = false;
  nh.param<bool>("useSquareThresh", use_square_thresh, use_square_thresh);
  pConfig->use_square_thresh = use_square_thresh;

  // Read extra iterations to do
  nh.param<int>("optExtraIterations", pConfig->opt_extra_iterations, 0);
  if (pConfig->opt_extra_iterations > 0)
    ROS_INFO("%d Extra iterations per optimization", pConfig->opt_extra_iterations);

  // Read in Odom noise values
  nh.param<double>("odom_xy_noise", pConfig->sigma_xy, 0.01);
  nh.param<double>("odom_theta_noise", pConfig->sigma_theta, 0.1);
  pConfig->sigma_theta *= PI / 180; // convert to radians

  // Read in AP noise values
  nh.param<double>("sigma_angle_bot", pConfig->sigma_angle_bot, 1);
  nh.param<double>("sigma_angle_AP", pConfig->sigma_angle_AP, 1);
  pConfig->sigma_angle_AP *= PI / 180; // convert to radians
  pConfig->sigma_angle_bot *= PI / 180; // convert to radians

  // Read in our Initialization noise values
  nh.param<double>("sigma_prior_rotation", pConfig->sigma_prior_rotation, 1.0e-4);
  nh.param<double>("sigma_prior_translation", pConfig->sigma_prior_translation, 1.0e-4);
  nh.param<double>("sigma_velocity", pConfig->sigma_velocity, 0.1);
  nh.param<double>("sigma_bias", pConfig->sigma_bias, 0.15);

  pConfig->sigma_prior_rotation *= PI / 180; // convert to radians

  // Read in optimization options
  nh.param<bool>("useISAM", pConfig->use_isam, true);
  nh.param<bool>("useBatch", pConfig->use_batch, false);
  nh.param<double>("optimizerLag", pConfig->optimizer_lag, 2.0);
  nh.param<double>("huberParam", pConfig->huber_param, 1.345);
  // Ensure that either both ISAM and Batch are false, reverting to Incremental optimization,
  // or either on of them is true.
  assert((! pConfig->use_isam && ! pConfig->use_batch) ||
         (pConfig->use_isam != pConfig->use_batch));
  nh.param<double>("relinearizeThreshold", pConfig->relinearizeThreshold, 0.01);
  nh.param<int>("relinearizeSkip",         pConfig->relinearizeSkip, 1);
  nh.param<bool>("cacheLinearizedFactors", pConfig->cacheLinearizedFactors, true);
  nh.param<bool>("findUnusedFactorSlots",  pConfig->findUnusedFactorSlots, true);
  nh.param<bool>("enablePartialRelinearizationCheck",
                 pConfig->enablePartialRelinearizationCheck, false);

  // File storage
  nh.param<std::string>("dataDir",          pConfig->data_dir, "/home/aarun/");
  nh.param<std::string>("graphFilename",    pConfig->graph_filename, "wifio_graph.dot");
  nh.param<bool>("shouldStoreFeatures",     pConfig->store_features, false);
  nh.param<std::string>("featuresFilename", pConfig->features_filename, "wifio_features.txt");
  if (pConfig->store_features) {
    pConfig->features_stream.open(pConfig->data_dir + pConfig->features_filename);
    time_t now = time(0);
    std::cout << "Storing Wifi odometery estimates in "
              << pConfig->data_dir + pConfig->features_filename << std::endl;
    pConfig->features_stream << "Wifi Features; Date/Time " << ctime(&now) << std::endl;
  }

  // Topic post-fix
  nh.param<std::string>("topicPostFix", pConfig->topic_post_fix, "");

  // Debug print to screen for the user
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  std::cout << "\t- fixed ID: " << pConfig->fixedId << std::endl;
  std::cout << "\t- gravity: " << pConfig->gravity.format(CommaInitFmt) << std::endl;

  std::cout << "Initialization:" << std::endl;
  std::cout << "\t- prior_ap: " << std::endl << pConfig->ap_pos_prior.transpose() << std::endl;

  std::cout << "Noise Parameters:" << std::endl;
  std::cout << "\t- sigma_prior_rotation: " << pConfig->sigma_prior_rotation << std::endl;
  std::cout << "\t- sigma_prior_translation: " << pConfig->sigma_prior_translation << std::endl;
  std::cout << "\t- sigma_velocity: " << pConfig->sigma_velocity << std::endl;
  std::cout << "\t- sigma_bias: " << pConfig->sigma_bias << std::endl;
  std::cout << "\t- sigma_angle_AP (deg): " << pConfig->sigma_angle_AP * 180 / PI << std::endl;
  std::cout << "\t- sigma_angle_bot (deg): " << pConfig->sigma_angle_bot * 180 / PI << std::endl;
  std::cout << "\t- AP's consider: ";
  print(pConfig->aps_to_include);
  std::cout << "\t- cam_to_odom: " << std::endl << pConfig->cam_to_odom << std::endl;
}

void print(std::unordered_set<int> const &s)
{
  std::copy(s.begin(),
            s.end(),
            std::ostream_iterator<int>(std::cout, " "));
  std::cout << std::endl;
}

void setup_subpub(ros::NodeHandle& nh) {

  // Subscribe to uv measurements
  subCamMeas = nh.subscribe("wifio/data_cam", 500,
                           handle_measurement_uv);
  ROS_INFO("UV Feat measurements, subscribing: %s", subCamMeas.getTopic().c_str());

  // Subsctibe to relative odometry measurement
  subRelOdomMeas = nh.subscribe("wifio/data_rel_odom", 2000,
                                 handle_measurement_odom);
  ROS_INFO("Relative Odom, subscribing: %s", subRelOdomMeas.getTopic().c_str());

  if (config->use_bot_side_angles) {
    subAoa = nh.subscribe(config->aoa_topic, 500, handle_measurement_single_aoa);
    ROS_INFO("Subscribing: %s", subAoa.getTopic().c_str());
  }

  // Camera/robot pose visualization
  pubPoseRobot = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("wifio/pose_robot/" + config->topic_post_fix,
                                                                        2);
  ROS_INFO("Publishing: %s", pubPoseRobot.getTopic().c_str());

  // AP pose visualization
  pubPoseAP = nh.advertise<visualization_msgs::Marker>("wifio/pose_ap/" + config->topic_post_fix, 2);
  ROS_INFO("Publishing: %s", pubPoseAP.getTopic().c_str());

  // robot path visualization
  pubPathRobot = nh.advertise<nav_msgs::Path>("wifio/path_robot/" + config->topic_post_fix, 2);
  ROS_INFO("Publishing: %s", pubPathRobot.getTopic().c_str());

  // Diff paths to be updated in camera graph
  pubDiffPath = nh.advertise<nav_msgs::Path>("wifio/data_odom/" + config->topic_post_fix, 2000);
  ROS_INFO("Publishing: %s", pubDiffPath.getTopic().c_str());

}

void handle_measurement_uv(const sensor_msgs::CompressedImage msg) {
  // Skip first keyframes
  optimize_graph(msg.header.stamp.toSec());
}

void handle_measurement_odom(const geometry_msgs::PoseStampedPtr& msg) {

  // Convert to eigen format
  Eigen::Vector3d relativePosition; // x, y, z
  Eigen::Vector4d relativeQuat; // w, x, y, z

  relativeQuat << msg->pose.orientation.w, msg->pose.orientation.x,
                  msg->pose.orientation.y, msg->pose.orientation.z;
  relativePosition << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if (config->store_features) {
    config->features_stream << std::fixed << "Odom Time: " << msg->header.stamp.toSec()
                            << " ; " << "Pos: " << relativePosition[0] <<  ", "
                                                << relativePosition[1] <<  ", "
                                                << relativePosition[2]
                            <<  "; " << "Quat: " << relativeQuat[0] << ", "
                                                 << relativeQuat[1] << ", "
                                                 << relativeQuat[2] << ", "
                                                 << relativeQuat[3] << std::endl;

  }
  // Send to graph solver
  graphsolver->addmeasurement_odom(msg->header.stamp.toSec(), relativePosition, relativeQuat);
}

double median(std::vector<double> a) {
  int n = a.size();
  if (n == 1)
      return a[0];
  // First we sort the array
  std::sort(a.begin(), a.end());
  // check for even case
  if (n % 2 != 0)
    return (double)a[n / 2];
  return (double)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
}

void handle_measurement_single_aoa(const rf_msgs::AoaPtr& msg) {
  double avg_rssi = 0;
  for (size_t i = 0; i < msg->rssi.size(); i++) avg_rssi += msg->rssi[i];
  avg_rssi /= msg->rssi.size();
  double ap_id = msg->txmac[5];

  double abs_aoa =  std::abs(msg->aoa[0]);
  if (config->use_square_thresh && avg_rssi != 0 && avg_rssi > config->rssi_thresh[msg->ap_id] &&
      (config->angle_thresh_square[0] < abs_aoa) && (abs_aoa > msg->aoa[0])) {

//    ROS_INFO("Passing AoA from AP %zu at time %f", ap_id, msg->header.stamp.toSec());
    graphsolver->addmeasurement_aoa(msg->header.stamp.toSec(), ap_id,
                                    msg->aoa[0], avg_rssi, true);

  } else if (!config->use_square_thresh && avg_rssi != 0 && avg_rssi > config->rssi_thresh[msg->ap_id] &&
             config->angle_thresh[0] < msg->aoa[0] &&
          msg->aoa[0] < config->angle_thresh[1]) {

//    ROS_INFO("Passing AoA from AP %zu at time %0.4f", ap_id, msg->header.stamp.toSec());
    graphsolver->addmeasurement_aoa(msg->header.stamp.toSec(), ap_id,
                                    msg->aoa[0], avg_rssi, true);

  }
}

void optimize_graph(double timestamp) {
  // Optimize graph
  graphsolver->optimize(timestamp);

  if (graphsolver->is_initialized()) {
    // Ros visualization
    gtsam::State state = graphsolver->get_current_state();
    publish_state(timestamp, state);

    publish_apMarkers(timestamp);

    bool isDiffTrajValid;
    std::vector<std::pair<double, gtsam::State>> trajectory = graphsolver->get_trajectory();
    std::vector<std::pair<double, gtsam::State>> diff_trajectory = graphsolver->get_diff_trajectory(isDiffTrajValid);

    publish_trajectory(timestamp, trajectory, pubPathRobot);

    if (isDiffTrajValid)
      publish_trajectory(timestamp, diff_trajectory, pubDiffPath);
  }
}

void publish_apMarkers(double timestamp) {
  visualization_msgs::Marker apMarkers;
  // Setup the markers
  apMarkers.header.frame_id = config->fixedId;
  apMarkers.header.stamp = ros::Time(timestamp);
  apMarkers.ns = "aps";
  apMarkers.action = visualization_msgs::Marker::ADD;
  apMarkers.pose.orientation.w = 1.0;
  apMarkers.id = 0;
  apMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
//  apMarkers.type = visualization_msgs::Marker::LINE_LIST;

  // Size the markers
  apMarkers.scale.x = 0.5;
  apMarkers.scale.y = 0.5;
  apMarkers.scale.z = 0.5;

  // Color the markers; red
  apMarkers.color.r = 1.0f;
  apMarkers.color.a = 1.0;

  double line_len = 1;

  for (const int &ii : config->aps_to_include ) {
    bool valid_pose = false;
    gtsam::Pose3 ap_pose = graphsolver->get_ap_pose(ii, valid_pose);
    if (valid_pose) {
      double yaw = ap_pose.rotation().yaw();
      // Push the start point of line
      geometry_msgs::Point p;
      p.x = ap_pose.x();
      p.y = ap_pose.y();
      p.z = 0; // AP positions are 2D for now
      apMarkers.points.push_back(p);
      //Push the end point of line
//      geometry_msgs::Point p_end;
//      p_end.x = p.x + line_len * cos(yaw);
//      p_end.y = p.y + line_len * sin(yaw);
//      p_end.z = 0;
//      apMarkers.points.push_back(p_end);

    }
  }

  pubPoseAP.publish(apMarkers);
}

void publish_state(double timestamp, gtsam::State& state) {

  // Return if we have not initialized yet
  if(!graphsolver->is_initialized())
      return;

  // Create our stamped pose with covariance
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.stamp = ros::Time(timestamp);
  pose.header.frame_id = config->fixedId;
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double,6,6>::Zero();
  ToPoseWithCovariance(state.pose(), covariance, pose.pose);

  // Publish this pose
  pubPoseRobot.publish(pose);
}

void publish_trajectory(double timestamp, Trajectory& trajectory, const ros::Publisher& pub) {

  // Return if trajectory is empty
  if (trajectory.empty()) {
    // ROS_WARN("Trajectory is empty");
    return;
  }

  // Create stamped pose for path publishing
  std::vector<geometry_msgs::PoseStamped> traj_est;
  for (auto & it : trajectory) {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time(it.first);
    poseStamped.header.seq = it.second.stateId();
    poseStamped.header.frame_id = config->fixedId;
    ToPose(it.second.pose(), poseStamped.pose);
    traj_est.push_back(poseStamped);
  }
  // Create pose arrays and publish
  nav_msgs::Path patharr;
  patharr.header.frame_id = config->fixedId;
  patharr.header.stamp = ros::Time(timestamp);
  patharr.header.seq = poses_seq++;
  patharr.poses = traj_est;
  pub.publish(patharr);
}

nav_msgs::Path get_trajectory(Trajectory& trajectory, bool& valid) {
  // Return if trajectory is empty
  if (trajectory.empty()) {
    // ROS_WARN("No trajectory to return; it is empty");
    nav_msgs::Path patharr;
    valid = false;
    return patharr;
  }

  // Create stamped pose for path publishing
  std::vector<geometry_msgs::PoseStamped> traj_est;
  for (auto & it : trajectory) {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time(it.first);
    poseStamped.header.seq = it.second.stateId();
    poseStamped.header.frame_id = config->fixedId;
    ToPose(it.second.pose(), poseStamped.pose);
    traj_est.push_back(poseStamped);
  }
  // Create pose arrays and publish
  nav_msgs::Path patharr;
  patharr.header.frame_id = config->fixedId;
  patharr.header.seq = poses_seq++;
  patharr.poses = traj_est;

  valid = true;
  return patharr;
}
