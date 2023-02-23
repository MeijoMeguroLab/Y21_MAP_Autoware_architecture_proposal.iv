/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #include <fstream>
std::ofstream ofs_NDT;
std::ofstream ofs_tp_max;

#include "ndt_scan_matcher/ndt_scan_matcher_core.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <thread>
#include <sys/stat.h>

#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ndt_scan_matcher/util_func.h"


NDTScanMatcher::NDTScanMatcher(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh),
  private_nh_(private_nh),
  tf2_listener_(tf2_buffer_),
  ndt_implement_type_(NDTImplementType::PCL_GENERIC),
  base_frame_("base_link"),
  ndt_base_frame_("ndt_base_link"),
  map_frame_("map"),
  converged_param_transform_probability_(4.5)
{
  key_value_stdmap_["state"] = "Initializing";

  int ndt_implement_type_tmp = 0;
  private_nh_.getParam("ndt_implement_type", ndt_implement_type_tmp);
  ndt_implement_type_ = static_cast<NDTImplementType>(ndt_implement_type_tmp);
  if (ndt_implement_type_ == NDTImplementType::PCL_GENERIC) {
    ROS_INFO("NDT Implement Type is PCL GENERIC");
    ndt_ptr_.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
  } else if (ndt_implement_type_ == NDTImplementType::PCL_MODIFIED) {
    ROS_INFO("NDT Implement Type is PCL MODIFIED");
    ndt_ptr_.reset(new NormalDistributionsTransformPCLModified<PointSource, PointTarget>);
  } else if (ndt_implement_type_ == NDTImplementType::OMP) {
    ROS_INFO("NDT Implement Type is OMP");

    std::shared_ptr<NormalDistributionsTransformOMP<PointSource, PointTarget>> ndt_omp_ptr(
      new NormalDistributionsTransformOMP<PointSource, PointTarget>);

    int search_method = static_cast<int>(omp_params_.search_method);
    private_nh_.getParam("omp_neighborhood_search_method", search_method);
    omp_params_.search_method = static_cast<ndt_omp::NeighborSearchMethod>(search_method);
    // TODO check search_method is valid value.
    
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);

    private_nh_.getParam("omp_num_threads", omp_params_.num_threads);
    omp_params_.num_threads = std::max(omp_params_.num_threads, 1);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);

    ndt_ptr_ = ndt_omp_ptr;
  } else {
    ndt_implement_type_ = NDTImplementType::PCL_GENERIC;
    ROS_INFO("NDT Implement Type is PCL GENERIC");
    ndt_ptr_.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
  }

  std::string mkdir_name =  "/home/megken/AutowareArchitectureProposal.proj/BAGs";
  mkdir(mkdir_name.c_str(),0777);
  std::string mkdir_name_ekf =  "/home/megken/AutowareArchitectureProposal.proj/BAGs/EKF";
  mkdir(mkdir_name_ekf.c_str(),0777);
  ofs_NDT.open("/home/megken/AutowareArchitectureProposal.proj/BAGs/judg.csv",std::ios_base::app);
  ofs_NDT
  <<"initial_points"
  <<","<<"ros_time"
  <<","<<"elapsed_time"
  <<","<<"exe_time"
  <<","<<"Warn_NDT"
  <<","<<"Warn_TP"
  <<","<<"Warn_iteration"
  <<","<<"Warn_exe_time"
  <<","<<"across_error"
  <<","<<"current_pose.x"
  <<","<<"current_pose.y"
   // <<","<<"current_pose.z"
  // <<","<<"current_pose.z"
  // <<","<<"transformation_probability"
  // <<","<<"covariance_xx"
  // <<","<<"covariance_xy"
  // <<","<<"covariance_yx"
  // <<","<<"covariance_yy"
  <<","<<"ellipse_short_radius" 
  <<","<<"center_TP"
  <<","<<"max_TP"
  <<","<<"min_TP"
  <<","<<"max_itr"
  <<","<<"ellipse center"
  <<","<<"ellipse hesse"
  <<","<<"ellipse_yaw"
  <<","<<"ellipse_long_radius" 
  <<","<<"stop_offset_ID"
  <<","<<"across_error"
  <<","<<"along_error"
  <<std::endl;

  ofs_tp_max.open("/home/megken/AutowareArchitectureProposal.proj/BAGs/tp_max.csv",std::ios_base::app);
  ofs_tp_max
  <<"initial_points"
  <<","<<"ros_time"
  <<","<<"elapsed_time"
  <<","<<"current_convergence_x"
  <<","<<"current_convergence_y"
  <<","<<"current_convergence_z"
  <<","<<"current_convergence_roll"
  <<","<<"current_convergence_pitch"
  <<","<<"current_convergence_yaw"
  <<","<<"convergence_x"
  <<","<<"convergence_y"
  <<","<<"convergence_z"
  <<","<<"max_TP"
  <<","<<"max_itr"
  <<std::endl;

  int points_queue_size = 0;
  private_nh_.getParam("input_sensor_points_queue_size", points_queue_size);
  points_queue_size = std::max(points_queue_size, 0);
  ROS_INFO("points_queue_size: %d", points_queue_size);

  private_nh_.getParam("base_frame", base_frame_);
  ROS_INFO("base_frame_id: %s", base_frame_.c_str());

  double trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  double step_size = ndt_ptr_->getStepSize();
  double resolution = ndt_ptr_->getResolution();
  int max_iterations = ndt_ptr_->getMaximumIterations();
  private_nh_.getParam("trans_epsilon", trans_epsilon);
  private_nh_.getParam("step_size", step_size);
  private_nh_.getParam("resolution", resolution);
  private_nh_.getParam("max_iterations", max_iterations);
  private_nh_.getParam("use_aoki_method", use_aoki_method);
  private_nh_.getParam("use_end_point", use_end_point);
  private_nh_.getParam("use_init_tp_correction", use_init_tp_correction);
  private_nh_.getParam("tp_threshold", tp_threshold);
  private_nh_.getParam("use_diff_tp_rate", use_diff_tp_rate);
  private_nh_.getParam("tp_rate_threshold", tp_rate_threshold);
  ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setResolution(resolution);
  ndt_ptr_->setMaximumIterations(max_iterations);
  ROS_INFO(
    "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
    step_size, resolution, max_iterations);

  private_nh_.getParam(
    "converged_param_transform_probability", converged_param_transform_probability_);

  initial_pose_sub_ =
    nh_.subscribe("ekf_pose_with_covariance", 100, &NDTScanMatcher::callbackInitialPose, this);
  map_points_sub_ = nh_.subscribe("pointcloud_map", 1, &NDTScanMatcher::callbackMapPoints, this);
  sensor_points_sub_ = nh_.subscribe("points_raw", 1, &NDTScanMatcher::callbackSensorPoints, this);
  twist_sub_ = nh_.subscribe("/vehicle/status/twist", 1, &NDTScanMatcher::twistcallback, this);

  sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ndt_pose_with_covariance", 10);
  ndt_pose_with_covariance_tp_max_pub_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ndt_pose_with_covariance_tp_max", 10);
  ndt_pose_with_covariance_tp_max_end_pub_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ndt_pose_with_covariance_tp_max_end", 10);
  ndt_pose_with_covariance_tp_max_threshold_pub_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ndt_pose_with_covariance_tp_max_threshold", 10);
  ndt_pose_with_covariance__evaluation_pub_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ndt_pose_with_covariance_evaluation", 10);
  initial_pose_with_covariance_pub_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initial_pose_with_covariance", 10);
  exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
  transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
  iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);
  initial_to_result_distance_pub_ =
    nh_.advertise<std_msgs::Float32>("initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    nh_.advertise<std_msgs::Float32>("initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    nh_.advertise<std_msgs::Float32>("initial_to_result_distance_new", 10);
  ndt_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    nh_.advertise<visualization_msgs::MarkerArray>("monte_carlo_initial_pose_marker", 10);
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

  service_ = nh_.advertiseService("ndt_align_srv", &NDTScanMatcher::serviceNDTAlign, this);
  // setup dynamic reconfigure server
  // f_ = boost::bind(&NDTScanMatcher::configCallback, this, _1, _2);
  // server_.setCallback(f_);

  diagnostic_thread_ = std::thread(&NDTScanMatcher::timerDiagnostic, this);
  diagnostic_thread_.detach();
}

NDTScanMatcher::~NDTScanMatcher() {}

void NDTScanMatcher::twistcallback(const geometry_msgs::TwistStamped::ConstPtr & twist_ptr){
  twist = *twist_ptr;
};

void NDTScanMatcher::timerDiagnostic()
{
  ros::Rate rate(100);
  while (ros::ok()) {
    diagnostic_msgs::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }

    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp = ros::Time::now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_.publish(diag_msg);

    rate.sleep();
  }
}

bool NDTScanMatcher::serviceNDTAlign(
  autoware_localization_srvs::PoseWithCovarianceStamped::Request & req,
  autoware_localization_srvs::PoseWithCovarianceStamped::Response & res)
{
  // get TF from pose_frame to map_frame
  geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
  getTransform(map_frame_, req.pose_with_cov.header.frame_id, TF_pose_to_map_ptr);

  // transform pose_frame to map_frame
  geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  tf2::doTransform(req.pose_with_cov, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    // TODO wait for map pointcloud
    return false;
  }

  if (ndt_ptr_->getInputSource() == nullptr) {
    // TODO wait for sensor pointcloud
    return false;
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  key_value_stdmap_["state"] = "Aligning";
  res.pose_with_cov = alignUsingMonteCarlo(ndt_ptr_, *mapTF_initial_pose_msg_ptr);
  key_value_stdmap_["state"] = "Sleeping";
  res.pose_with_cov.pose.covariance = req.pose_with_cov.pose.covariance;

  return true;
}

void NDTScanMatcher::callbackInitialPose(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr)
{
  // if rosbag restart, clear buffer
  if (!initial_pose_msg_ptr_array_.empty()) {
    if (initial_pose_msg_ptr_array_.front()->header.stamp > initial_pose_msg_ptr->header.stamp) {
      initial_pose_msg_ptr_array_.clear();
    }
  }

  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_msg_ptr_array_.push_back(initial_pose_msg_ptr);
  } else {
    // get TF from pose_frame to map_frame
    geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
    getTransform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(
      new geometry_msgs::PoseWithCovarianceStamped);
    tf2::doTransform(*initial_pose_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    initial_pose_msg_ptr_array_.push_back(mapTF_initial_pose_msg_ptr);
  }
}

void NDTScanMatcher::callbackMapPoints(
  const sensor_msgs::PointCloud2::ConstPtr & map_points_msg_ptr)
{
  const auto trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  const auto step_size = ndt_ptr_->getStepSize();
  const auto resolution = ndt_ptr_->getResolution();
  const auto max_iterations = ndt_ptr_->getMaximumIterations();

  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> new_ndt_ptr_;

  if (ndt_implement_type_ == NDTImplementType::PCL_GENERIC) {
    new_ndt_ptr_.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
  } else if (ndt_implement_type_ == NDTImplementType::PCL_MODIFIED) {
    new_ndt_ptr_.reset(new NormalDistributionsTransformPCLModified<PointSource, PointTarget>);
  } else if (ndt_implement_type_ == NDTImplementType::OMP) {
    std::shared_ptr<NormalDistributionsTransformOMP<PointSource, PointTarget>> ndt_omp_ptr(
      new NormalDistributionsTransformOMP<PointSource, PointTarget>);

    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);

    new_ndt_ptr_ = ndt_omp_ptr;
  } else {
    new_ndt_ptr_.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
  }

  new_ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  new_ndt_ptr_->setStepSize(step_size);
  new_ndt_ptr_->setResolution(resolution);
  new_ndt_ptr_->setMaximumIterations(max_iterations);

  pcl::PointCloud<PointTarget>::Ptr map_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  new_ndt_ptr_->setInputTarget(map_points_ptr);
  // create Thread
  // detach
  pcl::PointCloud<PointSource>::Ptr output_cloud(new pcl::PointCloud<PointSource>);
  new_ndt_ptr_->align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  ndt_map_mtx_.lock();
  ndt_ptr_ = new_ndt_ptr_;
  ndt_map_mtx_.unlock();
}

void NDTScanMatcher::callbackSensorPoints(
  const sensor_msgs::PointCloud2::ConstPtr & sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;
  if (first_time == true){
    first_sensor_time = sensor_ros_time.toSec();
    first_time = false;
  }

  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  // get TF base to sensor
  geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
  getTransform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);
  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();
  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(
    *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
  ndt_ptr_->setInputSource(sensor_points_baselinkTF_ptr);

  // check
  if (initial_pose_msg_ptr_array_.empty()) {
    ROS_WARN_STREAM_THROTTLE(1, "No Pose");
    return;
  }
  // searchNNPose using timestamp
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr initial_pose_old_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr initial_pose_new_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  getNearestTimeStampPose(
    initial_pose_msg_ptr_array_, sensor_ros_time, initial_pose_old_msg_ptr,
    initial_pose_new_msg_ptr);
  popOldPose(initial_pose_msg_ptr_array_, sensor_ros_time);
  // TODO check pose_timestamp - sensor_ros_time
  const auto initial_pose_msg =
    interpolatePose(*initial_pose_old_msg_ptr, *initial_pose_new_msg_ptr, sensor_ros_time);

  geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg;
  initial_pose_cov_msg.header = initial_pose_msg.header;
  initial_pose_cov_msg.pose.pose = initial_pose_msg.pose;

  if (ndt_ptr_->getInputTarget() == nullptr) {
    ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
    return;
  }

  // align
  Eigen::Affine3d initial_pose_affine;
  tf2::fromMsg(initial_pose_cov_msg.pose.pose, initial_pose_affine);
  const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
  Eigen::Matrix4f initial_pose_matrix_;
  Eigen::Matrix4f initial_pose_matrix_along;

  double align_time_sum = 0;
  double align_ros_time_sum = 0;
  double align_time_sum_along = 0;
  double align_ros_time_sum_along = 0;
  Eigen::Matrix4f center_result_pose_matrix(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f tp_max_matrix(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f tp_min_matrix(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f tp_max_end_matrix(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f tp_min_end_matrix(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f center_result_pose_matrix_along(Eigen::Matrix4f::Identity());
  std::vector<Eigen::Matrix4f> center_result_pose_matrix_array;
  std::vector<Eigen::Matrix4f> tp_max_matrix_array;
  std::vector<Eigen::Matrix4f> tp_min_matrix_array;
  std::vector<Eigen::Matrix4f> tp_max_end_matrix_array;
  std::vector<Eigen::Matrix4f> tp_min_end_matrix_array;
  std::vector<Eigen::Matrix4f> center_result_pose_matrix_array_along;
  Eigen::Matrix<double, 6, 6> center_result_cov_matrix;
  float center_transform_probability;
  size_t cener_iteration_num;
  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  int Warn_NDT=0;
  int Warn_TP=0;
  int Warn_iteration=0;
  int Warn_exe_time=0;
  int Warn_across_error=0;
  size_t itr_max=0;
  bool long_error_flag = false;

  float center_transform_probability_along;
  size_t cener_iteration_num_along;
  float tp_max_transform_probability;
  size_t tp_max_iteration_num;
  size_t tp_max_end_iteration_num;
  size_t tp_min_end_iteration_num;

  double initial_roll, ndt_pose_roll;
  double initial_pitch, ndt_pose_pitch;
  double initial_yaw, ndt_pose_yaw;
  double ellipse_yaw;
  double laplace_ellipse_yaw;
  double tp_max_yaw = 0, center_yaw = 0;
  double ndt_pose_roll_tmp,ndt_pose_pitch_tmp,ndt_pose_yaw_tmp;

  double A,B,C,across_error,along_error;
  Eigen::Matrix2d P;
  Eigen::Vector2d e;
  Eigen::MatrixXd along_tmp,across_tmp;

  double elapsed_time = sensor_ros_time.toSec() - first_sensor_time;
  std::string mkdir_name =  "/home/megken/AutowareArchitectureProposal.proj/BAGs/00_OUTPUT_.iv/";
  mkdir(mkdir_name.c_str(),0777);
  std::ofstream ofs_NDT_each;
  std::stringstream ss;
  std::string dir_name = "/home/megken/AutowareArchitectureProposal.proj/BAGs/00_OUTPUT_.iv/";
  boost::filesystem::create_directory(dir_name);
  ss << elapsed_time;
  ofs_NDT_each.open(dir_name +"/" + ss.str() + ".csv");
  ofs_NDT_each
  <<"sensor_ros_time"
  <<","<<"elapsed_time"
  <<","<<"x_offset"
  <<","<<"y_offset"
  <<","<<"z_offset"
  <<","<<"roll_offset"
  <<","<<"pitch_offset"
  <<","<<"yaw_offset"
  <<","<<"current_convergence_x"
  <<","<<"current_convergence_y"
  <<","<<"current_convergence_z"
  <<","<<"current_convergence_roll"
  <<","<<"current_convergence_pitch"
  <<","<<"current_convergence_yaw"
  <<","<<"convergence_x"
  <<","<<"convergence_y"
  <<","<<"convergence_z"
  <<","<<"ellipse_long_radius"
  <<","<<"ellipse_short_radius"
  <<","<<"ellipse_yaw"
  <<","<<"TP"
  <<","<<"iteration"
  <<","<<"align_time"
  <<","<<"across_error"
  <<","<<"along_error"
  <<std::endl;

  int exe_initial_points=0;
  std::vector<double> x_base_offset = {0};
  std::vector<double> y_base_offset = {0};
  int initial_point = x_base_offset.size();
  double diff_tp_rate = 1 - (tp_min_global / tp_max_global);
  double x_offset;
  double twist_hz = 50;
  double lidar_hz = 10;
  double sf = 0.015; // 0.015 = 1.5%
  double min_x_offset = 0.5;
  double stop_threshold_velocity = 2.7;
  int min_dr_cnt = abs(int((min_x_offset /(twist.twist.linear.x * sf)) * (twist_hz/lidar_hz))+1);
  double test = twist.twist.linear.x * sf * (lidar_hz/twist_hz);
  std::cout<<" min_dr_cnt " << min_dr_cnt << " , " << test << std::endl;
  if (use_aoki_method && !use_end_point){ // error
    x_base_offset.resize(11);
    y_base_offset.resize(11);
    x_base_offset = {0,0,0,0.5,-0.5,1.0,-1.0,1.5,-1.5,2.0,-2.0};
    y_base_offset = {0,0.3,-0.3,0,0,0,0,0,0,0,0};
    initial_point = x_base_offset.size();
  }else if(use_end_point){
    x_base_offset.resize(5);
    y_base_offset.resize(5);
    double lateral_offset = 0.3;
    y_base_offset = {0, lateral_offset, -lateral_offset, 0,0};
    x_base_offset[0] = 0;
    x_base_offset[1] = 0;
    x_base_offset[2] = 0;
    if (last_long_error_flag == false && diff_tp_rate > tp_rate_threshold){
      int test_dr_cnt = dr_cnt;
      dr_cnt = dr_cnt - min_dr_cnt;
      std::cout<<" offset reset " << " , " << test_dr_cnt << " , " << dr_cnt << std::endl;
    }
    if (dr_cnt <= min_dr_cnt){
      dr_cnt = min_dr_cnt;
    }
    if (twist.twist.linear.x < stop_threshold_velocity){
      dr_cnt = 1;
    }
    double stderr_vx = twist.twist.linear.x * sf * dr_cnt;
    // 1scan 0.22m(80km/h,SF1%)
    double sf_offset = stderr_vx * (lidar_hz/twist_hz);
    double x_offset_tmp = sf_offset;
    x_offset = x_offset_tmp > min_x_offset ? x_offset_tmp : min_x_offset;
    x_base_offset[3] = x_offset;
    x_base_offset[4] = -x_offset;
    // dr_cnt reset
    // if (x_offset_tmp > last_longitudinal_error){
    //   dr_cnt = abs(int((last_longitudinal_error /twist.twist.linear.x * sf) * (twist_hz/lidar_hz))+ 1);
    // }
    initial_point = x_base_offset.size();
    std::cout << std::setprecision(4) << "x_offset: "<< x_base_offset[3] << " , " << dr_cnt << " , " << sf_offset << " , " << last_longitudinal_error  << " , " << x_offset << std::endl;
  }
  double center_convergence_distance_x = 0;
  double center_convergence_distance_y = 0;
  double tp_max_convergence_distance_x = 0;
  double tp_max_convergence_distance_y = 0;
  double tp_max_end;
  double tp_min_end;
  Eigen::Matrix2f rot(Eigen::Matrix2f::Identity());
  Eigen::Matrix<double, 3, 3> covariance;

  std::cout<<"diff_tp_rate: " << diff_tp_rate << std::endl;
  if (use_init_tp_correction && diff_tp > tp_threshold && !use_diff_tp_rate && last_long_error_flag){ // Correct the initial position of the NDT search to the highest position of the previous TP
    std::cout<<"correction_init_pose: " << use_end_point << " , " << tp_threshold << " , " << diff_tp << " , " << correction_init_pose(0 ,3) << " , " << correction_init_pose(1 ,3) <<std::endl;
  }else if(use_init_tp_correction && diff_tp_rate > tp_rate_threshold && use_diff_tp_rate && last_long_error_flag){
    std::cout<<"correction_init_pose_rate: " << use_end_point << " , " << tp_rate_threshold << " , " << diff_tp_rate << " , " << correction_init_pose(0 ,3) << " , " << correction_init_pose(1 ,3) <<std::endl;

    // debug
    Eigen::Matrix4f test;
    double correction_init_pose_roll,correction_init_pose_pitch,correction_init_pose_yaw;
    double initial_pose_matrix_roll,initial_pose_matrix_pitch,initial_pose_matrix_yaw;
    double test_roll,test_pitch,test_yaw;
    test = initial_pose_matrix * correction_init_rotation + correction_init_pose;
    // std::cout << "correction_init_pose: " << correction_init_pose(0,0) << " , " << correction_init_pose(0,1) << " , " << correction_init_pose(0,2) << " , " << correction_init_pose(0,3) << " , " << correction_init_pose(1,0) << " , " << correction_init_pose(1,1) << " , " << correction_init_pose(1,2) << " , " << correction_init_pose(1,3) << " , " << correction_init_pose(2,0) << " , " << correction_init_pose(2,1) << " , " << correction_init_pose(2,2) << " , " << correction_init_pose(2,3) << std::endl;
    // std::cout << "initial_pose_matrix: " << initial_pose_matrix(0,0) << " , " << initial_pose_matrix(0,1) << " , " << initial_pose_matrix(0,2) << " , " << initial_pose_matrix(0,3) << " , " << initial_pose_matrix(1,0) << " , " << initial_pose_matrix(1,1) << " , " << initial_pose_matrix(1,2) << " , " << initial_pose_matrix(1,3) << " , " << initial_pose_matrix(2,0) << " , " << initial_pose_matrix(2,1) << " , " << initial_pose_matrix(2,2) << " , " << initial_pose_matrix(2,3) << std::endl;
    // std::cout << "test: " << test(0,0) << " , " << test(0,1) << " , " << test(0,2) << " , " << test(0,3) << " , " << test(1,0) << " , " << test(1,1) << " , " << test(1,2) << " , " << test(1,3) << " , " << test(2,0) << " , " << test(2,1) << " , " << test(2,2) << " , " << test(2,3) << std::endl;
    tf::Matrix3x3 correction_init_pose3;
    correction_init_pose3.setValue(static_cast<double>(correction_init_rotation(0, 0)), static_cast<double>(correction_init_rotation(0, 1)),
                            static_cast<double>(correction_init_rotation(0, 2)), static_cast<double>(correction_init_rotation(1, 0)),
                            static_cast<double>(correction_init_rotation(1, 1)), static_cast<double>(correction_init_rotation(1, 2)),
                            static_cast<double>(correction_init_rotation(2, 0)), static_cast<double>(correction_init_rotation(2, 1)),
                            static_cast<double>(correction_init_rotation(2, 2)));
    correction_init_pose3.getRPY(correction_init_pose_roll, correction_init_pose_pitch, correction_init_pose_yaw, 1);

    tf::Matrix3x3 initial_pose_matrix3;
    initial_pose_matrix3.setValue(static_cast<double>(initial_pose_matrix(0, 0)), static_cast<double>(initial_pose_matrix(0, 1)),
                            static_cast<double>(initial_pose_matrix(0, 2)), static_cast<double>(initial_pose_matrix(1, 0)),
                            static_cast<double>(initial_pose_matrix(1, 1)), static_cast<double>(initial_pose_matrix(1, 2)),
                            static_cast<double>(initial_pose_matrix(2, 0)), static_cast<double>(initial_pose_matrix(2, 1)),
                            static_cast<double>(initial_pose_matrix(2, 2)));
    initial_pose_matrix3.getRPY(initial_pose_matrix_roll, initial_pose_matrix_pitch, initial_pose_matrix_yaw, 1);

    tf::Matrix3x3 test3;
    test3.setValue(static_cast<double>(initial_pose_matrix(0, 0)), static_cast<double>(test(0, 1)),
                            static_cast<double>(test(0, 2)), static_cast<double>(test(1, 0)),
                            static_cast<double>(test(1, 1)), static_cast<double>(test(1, 2)),
                            static_cast<double>(test(2, 0)), static_cast<double>(test(2, 1)),
                            static_cast<double>(test(2, 2)));
    test3.getRPY(test_roll, test_pitch, test_yaw, 1);
    // std::cout << "correction_init_xy: " << correction_init_pose(0,3) << " , " << correction_init_pose(1,3)  << std::endl;
    // std::cout << "initial_xy: " << initial_pose_matrix(0,3) << " , " << initial_pose_matrix(1,3)  << std::endl;
    // std::cout << "test_xy: " << test(0,3) << " , " << test(1,3) << std::endl;
    // std::cout << "correction_init_rpy: " << correction_init_pose_roll << " , " << correction_init_pose_pitch << " , " << correction_init_pose_yaw << std::endl;
    // std::cout << "initial_rpy: " << initial_pose_matrix_roll << " , " << initial_pose_matrix_pitch << " , " << initial_pose_matrix_yaw << std::endl;
    // std::cout << "test_rpy: " << test_roll << " , " << test_pitch << " , " << test_yaw << std::endl;

  }
  //fast ConvergenceEvaluator
  int tp_max_id = 0;
  for (int i = 0; i<initial_point; i++)
  {
    exe_initial_points = i + 1;
    //reset
    if (use_init_tp_correction && diff_tp > tp_threshold && !use_diff_tp_rate && last_long_error_flag){ // Correct the initial position of the NDT search to the highest position of the previous TP
      initial_pose_matrix_ = initial_pose_matrix * correction_init_rotation + correction_init_pose;
    }else if (use_init_tp_correction && diff_tp_rate > tp_rate_threshold && use_diff_tp_rate && last_long_error_flag){
      initial_pose_matrix_ = initial_pose_matrix * correction_init_rotation + correction_init_pose;
    }else{
      initial_pose_matrix_ = initial_pose_matrix;
    }

    //offset
    Eigen::Vector2f base_offset_2d;
    base_offset_2d(0) =  x_base_offset[i];
    base_offset_2d(1) =  y_base_offset[i];
    Eigen::Vector2f offset_2d;
    offset_2d = rot*base_offset_2d;

    offset_2d(0) += center_convergence_distance_x;
    offset_2d(1) += center_convergence_distance_y;

    initial_pose_matrix_(0,3) += offset_2d(0);
    initial_pose_matrix_(1,3) += offset_2d(1);

      tf::Matrix3x3 initial_posture;  
      initial_posture.setValue(static_cast<double>(initial_pose_matrix_(0, 0)), static_cast<double>(initial_pose_matrix_(0, 1)),
                              static_cast<double>(initial_pose_matrix_(0, 2)), static_cast<double>(initial_pose_matrix_(1, 0)),
                              static_cast<double>(initial_pose_matrix_(1, 1)), static_cast<double>(initial_pose_matrix_(1, 2)),
                              static_cast<double>(initial_pose_matrix_(2, 0)), static_cast<double>(initial_pose_matrix_(2, 1)),
                              static_cast<double>(initial_pose_matrix_(2, 2)));
      initial_posture.getRPY(initial_roll, initial_pitch, initial_yaw, 1);

    pcl::PointCloud<PointSource>::Ptr output_cloud(new pcl::PointCloud<PointSource>);
    const auto align_start_ros_time = ros::Time::now();
    const auto align_start_time = std::chrono::system_clock::now();
    key_value_stdmap_["state"] = "Aligning";
    ndt_ptr_->align(*output_cloud, initial_pose_matrix_);
    // pcl::io::savePCDFileBinary(dir_name +"/" + std::to_string(i) + ".pcd"  , *output_cloud);
    key_value_stdmap_["state"] = "Sleeping";
    const auto align_end_time = std::chrono::system_clock::now();
    const auto align_end_ros_time = ros::Time::now();
    const double align_time =
      std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time)
        .count() /
      1000.0;
    align_time_sum += align_time;
    double align_ros_time = align_end_ros_time.nsec - align_start_ros_time.nsec;
    align_ros_time_sum += align_ros_time;

    const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
    const std::vector<Eigen::Matrix4f> result_pose_matrix_array = ndt_ptr_->getFinalTransformationArray();
    const float transform_probability = ndt_ptr_->getTransformationProbability();
    const size_t iteration_num = ndt_ptr_->getFinalNumIteration();

    if(iteration_num > itr_max)
    {
      itr_max = iteration_num;
    }

    tp_tmp = transform_probability;
    //NDT from ekf initial pose
    if (i == 0) // center_result
    {
      center_result_pose_matrix = result_pose_matrix;
      center_result_pose_matrix_array = result_pose_matrix_array;

      center_convergence_distance_x =  center_result_pose_matrix(0,3) - initial_pose_matrix_(0,3);
      center_convergence_distance_y =  center_result_pose_matrix(1,3) - initial_pose_matrix_(1,3);

      tf::Matrix3x3 result_posture;  
      result_posture.setValue(static_cast<double>(center_result_pose_matrix(0, 0)), static_cast<double>(center_result_pose_matrix(0, 1)),
                              static_cast<double>(center_result_pose_matrix(0, 2)), static_cast<double>(center_result_pose_matrix(1, 0)),
                              static_cast<double>(center_result_pose_matrix(1, 1)), static_cast<double>(center_result_pose_matrix(1, 2)),
                              static_cast<double>(center_result_pose_matrix(2, 0)), static_cast<double>(center_result_pose_matrix(2, 1)),
                              static_cast<double>(center_result_pose_matrix(2, 2)));
      result_posture.getRPY(ndt_pose_roll, ndt_pose_pitch, ndt_pose_yaw, 1);

      //~Hesse matrix calculate~
      Eigen::Matrix<double, 6, 6> result_cov_matrix = ndt_ptr_->getCovariance();

      Eigen::Matrix2d LaplaceApp_2d;
      LaplaceApp_2d(0, 0) = result_cov_matrix(0, 0);
      LaplaceApp_2d(0, 1) = result_cov_matrix(0, 1);
      LaplaceApp_2d(1, 0) = result_cov_matrix(1, 0);
      LaplaceApp_2d(1, 1) = result_cov_matrix(1, 1);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> laplace_eigensolver(LaplaceApp_2d);
      const Eigen::Vector2d pc_vector_laplace = laplace_eigensolver.eigenvectors().col(1);
      laplace_ellipse_yaw = std::atan2(pc_vector_laplace.y(), pc_vector_laplace.x());
      rot = Eigen::Rotation2Df(laplace_ellipse_yaw);
      center_transform_probability = transform_probability;
      cener_iteration_num = iteration_num;     
      center_yaw = ndt_pose_yaw;

      tp_max = transform_probability; // init
      tp_min = transform_probability; // init
      tp_max_matrix = result_pose_matrix; // init
      tp_min_matrix = result_pose_matrix; // init
      tp_max_yaw = ndt_pose_yaw; // init
      
      if (
        cener_iteration_num >= ndt_ptr_->getMaximumIterations() + 2 ||
        center_transform_probability < converged_param_transform_probability_
        )
      {
      is_converged = false;
      ++skipping_publish_num;
      std::cout<<"WARN not using NDT"<<std::endl;
      ROS_WARN("Not Converged!");
      break;
      } else 
      {
        skipping_publish_num = 0;
      }
    }else{
      tf::Matrix3x3 result_posture_tmp;
      result_posture_tmp.setValue(static_cast<double>(result_pose_matrix(0, 0)), static_cast<double>(result_pose_matrix(0, 1)),
                              static_cast<double>(result_pose_matrix(0, 2)), static_cast<double>(result_pose_matrix(1, 0)),
                              static_cast<double>(result_pose_matrix(1, 1)), static_cast<double>(result_pose_matrix(1, 2)),
                              static_cast<double>(result_pose_matrix(2, 0)), static_cast<double>(result_pose_matrix(2, 1)),
                              static_cast<double>(result_pose_matrix(2, 2)));
      result_posture_tmp.getRPY(ndt_pose_roll_tmp, ndt_pose_pitch_tmp, ndt_pose_yaw_tmp, 1);
    }

    //Judgment
    double align_time_sum_max = 80;
    if(
    transform_probability < converged_param_transform_probability_ 
    ||iteration_num >= ndt_ptr_->getMaximumIterations() + 2 
    ||align_time_sum > align_time_sum_max
    )
    {
      if (transform_probability < converged_param_transform_probability_)
      {
        Warn_TP++;
        std::cout<<"WARN TP"<<std::endl;
      }
      else if (iteration_num >= ndt_ptr_->getMaximumIterations() + 2) 
      {
        Warn_iteration++;
        std::cout<<"WARN Iteration"<<std::endl;
      } 
      else if (align_time_sum > align_time_sum_max)
      {
        Warn_exe_time++;
        std::cout<<"WARN exe_time"<<std::endl;
      }
    Warn_NDT = 1;
    //break;
    }else{
      if (tp_max < tp_tmp)
      {
        tp_max = tp_tmp;
        tp_max_matrix = result_pose_matrix;
        tp_max_matrix_array = result_pose_matrix_array;
        tp_max_iteration_num = iteration_num;
        tp_max_yaw = ndt_pose_yaw_tmp;
        tp_max_id = i;
      }

      if (tp_min > tp_tmp)
      {
        tp_min = tp_tmp;
        tp_min_matrix = result_pose_matrix;
        tp_min_matrix_array = result_pose_matrix_array;
      }

      // use center TP and both ends(use_aoki_method and use_end_point)
      if (i % 2 == 1 && center_transform_probability < tp_tmp){
        tp_max_end = tp_tmp;
        tp_max_end_matrix = result_pose_matrix;
        tp_max_end_matrix_array = result_pose_matrix_array;
        tp_max_end_iteration_num = iteration_num;
        tp_max_yaw = ndt_pose_yaw_tmp;
        tp_min_end = center_transform_probability;
        tp_min_end_matrix = center_result_pose_matrix;
        tp_min_end_matrix_array = center_result_pose_matrix_array;
        tp_min_end_iteration_num = cener_iteration_num;
      }else if(i % 2 == 1 && center_transform_probability > tp_tmp){
        tp_max_end = center_transform_probability;
        tp_max_end_matrix = center_result_pose_matrix;
        tp_max_end_matrix_array = center_result_pose_matrix_array;
        tp_max_end_iteration_num = cener_iteration_num;
        tp_min_end = tp_tmp;
        tp_min_end_matrix = result_pose_matrix;
        tp_min_end_matrix_array = result_pose_matrix_array;
        tp_min_end_iteration_num = iteration_num;
      }

      // use center TP and both ends(use_end_point)
      if (i % 2 == 0 && tp_max_end < tp_tmp){
        tp_max_end = tp_tmp;
        tp_max_end_matrix = result_pose_matrix;
        tp_max_end_matrix_array = result_pose_matrix_array;
        tp_max_end_iteration_num = iteration_num;
        tp_max_yaw = ndt_pose_yaw_tmp;
      }else if(i % 2 == 0 && tp_min_end > tp_tmp){
        tp_min_end = tp_tmp;
        tp_min_end_matrix = result_pose_matrix;
        tp_min_end_matrix_array = result_pose_matrix_array;
        tp_min_end_iteration_num = iteration_num;
      }
    }

    if (i < 3 && tp_tmp > center_transform_probability){
      long_error_flag = true;
    }

    // Ellipse calculation
    Eigen::Vector3d p3d(result_pose_matrix(0,3), result_pose_matrix(1,3),0);
    tmp_centroid_ += p3d;
    tmp_cov_ += p3d * p3d.transpose();
    Eigen::Vector3d centroid = tmp_centroid_ / exe_initial_points;
    covariance = (tmp_cov_ / exe_initial_points - (centroid * centroid.transpose()));
    //covariance *= (exe_initial_points -1) / (exe_initial_points);
    // identity matrix is minimam dispersion
    Eigen::Matrix<double, 3, 3> identity(Eigen::Matrix<double, 3, 3>::Identity());
    covariance = covariance + (0.0025 * identity);

    Eigen::Matrix2d cov_2d;
    cov_2d(0, 0) = covariance(0, 0);
    cov_2d(0, 1) = covariance(0, 1);
    cov_2d(1, 0) = covariance(1, 0);
    cov_2d(1, 1) = covariance(1, 1);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov_2d);
    ellipse_long_radius = std::sqrt(eigensolver.eigenvalues()(1) * chi);
    ellipse_short_radius = std::sqrt(eigensolver.eigenvalues()(0) * chi);
    const Eigen::Vector2d pc_vector = eigensolver.eigenvectors().col(1);
    ellipse_yaw = std::atan2(pc_vector.y(), pc_vector.x());

    A = (std::pow(ellipse_short_radius*cos(ellipse_yaw),2) + std::pow(ellipse_long_radius*sin(ellipse_yaw),2)) / std::pow(ellipse_long_radius*ellipse_short_radius,2);
    B = (std::pow(ellipse_short_radius,2) * cos(ellipse_yaw)*sin(ellipse_yaw) - std::pow(ellipse_long_radius,2) * cos(ellipse_yaw)*sin(ellipse_yaw))/std::pow(ellipse_long_radius*ellipse_short_radius,2);
    C = (std::pow(ellipse_short_radius*sin(ellipse_yaw),2) + std::pow(ellipse_long_radius*cos(ellipse_yaw),2)) / std::pow(ellipse_long_radius*ellipse_short_radius,2);
    P << A,B, B,C;
    e << cos(ndt_pose_yaw),sin(ndt_pose_yaw);
    across_tmp = e.transpose()*P*e/P.determinant();
    across_error = std::sqrt(across_tmp(0,0));
    e << cos(ndt_pose_yaw + M_PI/2),sin(ndt_pose_yaw + M_PI/2);
    along_tmp = e.transpose()*P*e/P.determinant();
    along_error = std::sqrt(along_tmp(0,0));

    if(across_error > 0.3)
    {
      Warn_across_error++;
      std::cout<<"WARN across_error"<<std::endl;
    }

    // save convergence result in each pose
     ofs_NDT_each
     <<std::fixed<<std::setprecision(10)<<sensor_ros_time
     <<","<<elapsed_time
     <<","<<initial_pose_matrix_(0,3)
     <<","<<initial_pose_matrix_(1,3)
     <<","<<initial_pose_matrix_(2,3)
     <<","<<initial_roll
     <<","<<initial_pitch
     <<","<<initial_yaw
     <<","<<center_result_pose_matrix(0, 3)
     <<","<<center_result_pose_matrix(1, 3)
     <<","<<center_result_pose_matrix(2, 3)
     <<","<<ndt_pose_roll
     <<","<<ndt_pose_pitch
     <<","<<ndt_pose_yaw
     <<","<<result_pose_matrix(0,3)
     <<","<<result_pose_matrix(1,3)
     <<","<<result_pose_matrix(2,3)
     <<","<<ellipse_long_radius
     <<","<<ellipse_short_radius
     <<","<<ellipse_yaw
     <<","<<transform_probability
     <<","<<iteration_num
     <<","<<align_time
     <<","<<across_error
     <<","<<along_error
     << std::endl;

    // Decide whether to continue the calculation
    // if the ellipse are small enough, it break
    if (!use_end_point){
      if(stop_offset_ID==0 && i==4 && ellipse_long_radius < 0.2)
      {
        break;
      }

      if(stop_offset_ID<=1 && i==6 && ellipse_long_radius < 0.5)
      {
        break;
      }

      if(stop_offset_ID<=2 && i==8 && ellipse_long_radius < 1.0)
      {
        break;
      }
    }

  }

  if (use_end_point && !use_diff_tp_rate){
    diff_tp = tp_max_end - tp_min_end;
    tp_max_global = tp_max_end;
    tp_min_global = tp_min_end;
  }else if(use_end_point && use_diff_tp_rate){
    tp_max_global = tp_max_end;
    tp_min_global = tp_min_end;
  }else if(!use_end_point && !use_diff_tp_rate){
    diff_tp = tp_max - tp_min;
    tp_max_global = tp_max;
    tp_min_global = tp_min;
  }else if(!use_end_point && use_diff_tp_rate){
    tp_max_global = tp_max;
    tp_min_global = tp_min;
  }
  correction_init_pose(0 ,3) = tp_max_matrix(0, 3) - center_result_pose_matrix(0, 3);
  correction_init_pose(1 ,3) = tp_max_matrix(1, 3) - center_result_pose_matrix(1, 3);
  bool offset_yaw = false;
  double theta = center_yaw - tp_max_yaw;
  if (offset_yaw && theta != 0){
    correction_init_rotation(0,0) = cos(theta);
    correction_init_rotation(0,1) = -sin(theta);
    correction_init_rotation(1,0) = sin(theta);
    correction_init_rotation(1,1) = cos(theta);
    correction_init_rotation(2,2) = 1;
    correction_init_rotation(3,3) = 1;
    std::cout << "theta: " << theta << " , " << cos(theta) << " , " << sin(theta) << std::endl;
  }else{
    correction_init_rotation(0,0) = 1;
    correction_init_rotation(1,1) = 1;
    correction_init_rotation(2,2) = 1;
    correction_init_rotation(3,3) = 1;
    // std::cout << "theta: " << cos(theta) << " , " << sin(theta) << std::endl;
  }
  if (long_error_flag == false){  // center_TP > end_TP
    last_long_error_flag = false;
  }else{
    last_long_error_flag = true;
  }
  if (x_offset > along_error ){
    if (dr_cnt > min_dr_cnt && long_error_flag == false){
      dr_cnt --;
    }
  }else{
    dr_cnt ++;
  }
  last_longitudinal_error = along_error;
  last_lateral_error = across_error;


  //Determines the minimum number of offsets for the next scan
  if(is_converged)
  {
    if(ellipse_long_radius < 0.2)
    {
      stop_offset_ID=0;
    }
    else if(ellipse_long_radius >=0.2 && ellipse_long_radius < 0.5)
    {
      stop_offset_ID=1;
    }
    else if(ellipse_long_radius >=0.5 && ellipse_long_radius < 1.0)
    {
      stop_offset_ID=2;
    }
    else if(ellipse_long_radius >=1.0)
    {
      stop_offset_ID=3;
    }
  }else
  {
  // If the offset of the initial search position is widened too much in such a state, the number of iterations and the processing time will increase. 
  // So the next scan start at the small offset of the initial search position
    // stop_offset_ID=0;
  }

  std::cout << "time:" << align_time_sum<<  std::endl;
  std::cout<<"late-long "<<across_error << " , " << along_error <<std::endl;
  if (!use_end_point){
    std::cout<<"Offset_ID "<<stop_offset_ID << " , " << tp_max_id <<std::endl;
  }
  std::cout<<"-------------------------------"<<std::endl;

  // Substitute the final covariance
  center_result_cov_matrix(0,0) = covariance(0, 0);
  center_result_cov_matrix(0,1) = covariance(0, 1);
  center_result_cov_matrix(1,0) = covariance(1, 0);
  center_result_cov_matrix(1,1) = covariance(1, 1);

  // finish ConvergenceEvaluator
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = center_result_pose_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  // finish ConvergenceEvaluator TP max
  Eigen::Affine3d result_pose_affine_tp_max;
  result_pose_affine_tp_max.matrix() = tp_max_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg_tp_max = tf2::toMsg(result_pose_affine_tp_max);

  std::vector<geometry_msgs::Pose> result_pose_msg_array;
  for (const auto & pose_matrix : center_result_pose_matrix_array) {
    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose_matrix.cast<double>();
    const geometry_msgs::Pose pose_msg = tf2::toMsg(pose_affine);
    result_pose_msg_array.push_back(pose_msg);
  }

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() /
    1000.0;

  //rest
  tmp_centroid_(0, 0) = 0;
  tmp_centroid_(1, 0) = 0;
  tmp_centroid_(2, 0) = 0;
  tmp_cov_(0, 0) = 0;
  tmp_cov_(0, 1) = 0;
  tmp_cov_(0, 2) = 0;
  tmp_cov_(1, 0) = 0;
  tmp_cov_(1, 1) = 0;
  tmp_cov_(1, 2) = 0;
  tmp_cov_(2, 0) = 0;
  tmp_cov_(2, 1) = 0;
  tmp_cov_(2, 2) = 0;

  // publish
  geometry_msgs::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;

  geometry_msgs::PoseWithCovarianceStamped result_pose_with_cov_msg_tp_max;
  result_pose_with_cov_msg_tp_max.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg_tp_max.header.frame_id = map_frame_;
  result_pose_with_cov_msg_tp_max.pose.pose = result_pose_msg_tp_max;

  geometry_msgs::PoseWithCovarianceStamped result_pose_with_cov_msg_tp_max_end;
  result_pose_with_cov_msg_tp_max_end.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg_tp_max_end.header.frame_id = map_frame_;
  result_pose_with_cov_msg_tp_max_end.pose.pose = result_pose_msg_tp_max;

  // //TODO temporary value
  result_pose_with_cov_msg.pose.covariance[0] = center_result_cov_matrix(0,0); // x - x
  result_pose_with_cov_msg.pose.covariance[1] = center_result_cov_matrix(0,1); // x - y
  result_pose_with_cov_msg.pose.covariance[6] = center_result_cov_matrix(1,0); // y - x
  result_pose_with_cov_msg.pose.covariance[7] = center_result_cov_matrix(1,1); // y - y

  result_pose_with_cov_msg.pose.covariance[5 * 6 + 5] = 0.000625; //yaw-yaw

  // result_pose_with_cov_msg.pose.covariance[5] = center_result_cov_matrix(0,5); //x-yaw
  // result_pose_with_cov_msg.pose.covariance[11] = center_result_cov_matrix(1,5); //y-yaw
  // result_pose_with_cov_msg.pose.covariance[30] = center_result_cov_matrix(5,0); //yaw-x
  // result_pose_with_cov_msg.pose.covariance[31] = center_result_cov_matrix(5,1); //yaw-y
  // result_pose_with_cov_msg.pose.covariance[35] = center_result_cov_matrix(5,5)*500; //yaw-yaw

  result_pose_with_cov_msg_tp_max.pose.covariance[0] = center_result_cov_matrix(0,0); // x - x
  result_pose_with_cov_msg_tp_max.pose.covariance[1] = center_result_cov_matrix(0,1); // x - y
  result_pose_with_cov_msg_tp_max.pose.covariance[6] = center_result_cov_matrix(1,0); // y - x
  result_pose_with_cov_msg_tp_max.pose.covariance[7] = center_result_cov_matrix(1,1); // y - y

  result_pose_with_cov_msg_tp_max.pose.covariance[5 * 6 + 5] = 0.000625; //yaw-yaw

  result_pose_with_cov_msg_tp_max_end.pose.covariance[0] = center_result_cov_matrix(0,0); // x - x
  result_pose_with_cov_msg_tp_max_end.pose.covariance[1] = center_result_cov_matrix(0,1); // x - y
  result_pose_with_cov_msg_tp_max_end.pose.covariance[6] = center_result_cov_matrix(1,0); // y - x
  result_pose_with_cov_msg_tp_max_end.pose.covariance[7] = center_result_cov_matrix(1,1); // y - y

  result_pose_with_cov_msg_tp_max_end.pose.covariance[5 * 6 + 5] = 0.000625; //yaw-yaw

  // result_pose_with_cov_msg.pose.covariance[0] = 0.025; // x - x
  // result_pose_with_cov_msg.pose.covariance[7] = 0.025; // y - y
  // result_pose_with_cov_msg.pose.covariance[2 * 6 + 2] = 0.025;
  // result_pose_with_cov_msg.pose.covariance[3 * 6 + 3] = 0.000625;
  // result_pose_with_cov_msg.pose.covariance[4 * 6 + 4] = 0.000625;
  // result_pose_with_cov_msg.pose.covariance[5 * 6 + 5] = 0.000625; //yaw-yaw

  if (is_converged) {
    ndt_pose_pub_.publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_.publish(result_pose_with_cov_msg);
    ndt_pose_with_covariance_tp_max_pub_.publish(result_pose_with_cov_msg_tp_max);
    ndt_pose_with_covariance_tp_max_end_pub_.publish(result_pose_with_cov_msg_tp_max_end);
    if (use_init_tp_correction && diff_tp > tp_threshold){
      ndt_pose_with_covariance_tp_max_threshold_pub_.publish(result_pose_with_cov_msg_tp_max);
    }else if (diff_tp > tp_threshold){
      ndt_pose_with_covariance_tp_max_threshold_pub_.publish(result_pose_with_cov_msg_tp_max);
    }else{
      ndt_pose_with_covariance_tp_max_threshold_pub_.publish(result_pose_with_cov_msg);
    }

    ofs_NDT
    <<exe_initial_points
    <<","<<sensor_ros_time
    <<","<<elapsed_time
    <<","<<align_time_sum
    <<","<<Warn_NDT
    <<","<<Warn_TP
    <<","<<Warn_iteration
    <<","<<Warn_exe_time
    <<","<<Warn_across_error
    <<","<<std::fixed<<std::setprecision(10)<<center_result_pose_matrix(0, 3)
    <<","<<center_result_pose_matrix(1, 3)
  // <<","<<center_result_pose_matrix(2, 3)
  // <<","<<center_transform_probability
  // <<","<<center_result_cov_matrix(0,0)
  // <<","<<center_result_cov_matrix(0,1)
  // <<","<<center_result_cov_matrix(1,0)
  // <<","<<center_result_cov_matrix(1,1)
  <<","<<ellipse_short_radius
  <<","<<center_transform_probability
  <<","<<tp_max
  <<","<<tp_min
  <<","<<itr_max
  <<","<<ndt_pose_yaw
  <<","<<laplace_ellipse_yaw
  <<","<<ellipse_yaw
  <<","<<ellipse_long_radius
  <<","<<stop_offset_ID
  <<","<<across_error
  <<","<<along_error
  << std::endl;

  ofs_tp_max
  <<exe_initial_points
  <<","<<std::fixed<<std::setprecision(10)<<sensor_ros_time
  <<","<<elapsed_time
  <<","<<initial_pose_matrix_(0,3)
  <<","<<initial_pose_matrix_(1,3)
  <<","<<initial_pose_matrix_(2,3)
  <<","<<initial_roll
  <<","<<initial_pitch
  <<","<<initial_yaw
  <<","<<std::fixed<<std::setprecision(10)<<center_result_pose_matrix(0, 3)
  <<","<<center_result_pose_matrix(1, 3)
  <<","<<std::fixed<<std::setprecision(10)<<tp_max_matrix(0, 3)
  <<","<<tp_max_matrix(1, 3)
  <<","<<tp_max_matrix(2, 3)
  <<","<<tp_max
  <<","<<tp_max_iteration_num
  << std::endl;
  }else{
    std::cout<<"NDT not converged" <<std::endl;
  }
  ndt_pose_with_covariance__evaluation_pub_.publish(result_pose_with_cov_msg);

  publishTF(map_frame_, ndt_base_frame_, result_pose_stamped_msg);

  pcl::PointCloud<PointSource>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, center_result_pose_matrix);
  sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame_;
  sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);

  initial_pose_with_covariance_pub_.publish(initial_pose_cov_msg);

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::Marker::ADD;
  for (const auto & pose_msg : result_pose_msg_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = ExchangeColorCrc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }
  // TODO delete old marker
  for (; i < ndt_ptr_->getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::Pose();
    marker.color = ExchangeColorCrc(0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_.publish(marker_array);

  std_msgs::Float32 exe_time_msg;
  exe_time_msg.data = exe_time;
  // exe_time_msg.data = align_time_sum;
  exe_time_pub_.publish(exe_time_msg);

  std_msgs::Float32 transform_probability_msg;
  transform_probability_msg.data = center_transform_probability;
  // transform_probability_msg.data = tp_max;
  transform_probability_pub_.publish(transform_probability_msg);

  std_msgs::Float32 iteration_num_msg;
  iteration_num_msg.data = cener_iteration_num;
  // iteration_num_msg.data = itr_max;
  iteration_num_pub_.publish(iteration_num_msg);

  std_msgs::Float32 initial_to_result_distance_msg;
  initial_to_result_distance_msg.data = std::sqrt(
    std::pow(
      initial_pose_cov_msg.pose.pose.position.x - result_pose_with_cov_msg.pose.pose.position.x,
      2.0) +
    std::pow(
      initial_pose_cov_msg.pose.pose.position.y - result_pose_with_cov_msg.pose.pose.position.y,
      2.0) +
    std::pow(
      initial_pose_cov_msg.pose.pose.position.z - result_pose_with_cov_msg.pose.pose.position.z,
      2.0));
  initial_to_result_distance_pub_.publish(initial_to_result_distance_msg);

  std_msgs::Float32 initial_to_result_distance_old_msg;
  initial_to_result_distance_old_msg.data = std::sqrt(
    std::pow(
      initial_pose_old_msg_ptr->pose.pose.position.x -
        result_pose_with_cov_msg.pose.pose.position.x,
      2.0) +
    std::pow(
      initial_pose_old_msg_ptr->pose.pose.position.y -
        result_pose_with_cov_msg.pose.pose.position.y,
      2.0) +
    std::pow(
      initial_pose_old_msg_ptr->pose.pose.position.z -
        result_pose_with_cov_msg.pose.pose.position.z,
      2.0));
  initial_to_result_distance_old_pub_.publish(initial_to_result_distance_old_msg);

  std_msgs::Float32 initial_to_result_distance_new_msg;
  initial_to_result_distance_new_msg.data = std::sqrt(
    std::pow(
      initial_pose_new_msg_ptr->pose.pose.position.x -
        result_pose_with_cov_msg.pose.pose.position.x,
      2.0) +
    std::pow(
      initial_pose_new_msg_ptr->pose.pose.position.y -
        result_pose_with_cov_msg.pose.pose.position.y,
      2.0) +
    std::pow(
      initial_pose_new_msg_ptr->pose.pose.position.z -
        result_pose_with_cov_msg.pose.pose.position.z,
      2.0));
  initial_to_result_distance_new_pub_.publish(initial_to_result_distance_new_msg);

  key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
  key_value_stdmap_["transform_probability"] = std::to_string(center_transform_probability);
  key_value_stdmap_["iteration_num"] = std::to_string(cener_iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

}

geometry_msgs::PoseWithCovarianceStamped NDTScanMatcher::alignUsingMonteCarlo(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
  const geometry_msgs::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    ROS_WARN("No Map or Sensor PointCloud");
    return geometry_msgs::PoseWithCovarianceStamped();
  }

  // generateParticle
  const auto initial_pose_array = createRandomPoseArray(initial_pose_with_cov, 100);

  std::vector<Particle> particle_array;
  pcl::PointCloud<PointSource>::Ptr output_cloud(new pcl::PointCloud<PointSource>);

  int i = 0;
  for (const auto & initial_pose : initial_pose_array.poses) {
    Eigen::Affine3d initial_pose_affine;
    tf2::fromMsg(initial_pose, initial_pose_affine);
    const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

    ndt_ptr->align(*output_cloud, initial_pose_matrix);

    const Eigen::Matrix4f result_pose_matrix = ndt_ptr->getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::Pose result_pose = tf2::toMsg(result_pose_affine);

    const auto transform_probability = ndt_ptr->getTransformationProbability();
    const auto num_iteration = ndt_ptr->getFinalNumIteration();

    Particle particle(initial_pose, result_pose, transform_probability, num_iteration);
    particle_array.push_back(particle);
    publishMarkerForDebug(particle, i++);

    pcl::PointCloud<PointSource>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<PointSource>);
    const auto sensor_points_baselinkTF_ptr = ndt_ptr->getInputSource();
    pcl::transformPointCloud(
      *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = initial_pose_with_cov.header.stamp;
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;
  // ndt_pose_with_covariance_pub_.publish(result_pose_with_cov_msg);

  return result_pose_with_cov_msg;
}

void NDTScanMatcher::publishMarkerForDebug(const Particle & particle, const size_t i)
{
  // TODO getNumSubscribers
  // TODO clear old object
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.id = i;

  marker.ns = "initial_pose_transform_probability_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_iteration_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_index_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc((1.0 * i) / 100);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_transform_probability_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_iteration_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_index_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc((1.0 * i) / 100);
  marker_array.markers.push_back(marker);

  ndt_monte_carlo_initial_pose_marker_pub_.publish(marker_array);
}

void NDTScanMatcher::publishTF(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::PoseStamped & pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}

bool NDTScanMatcher::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr, const ros::Time & time_stamp)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp, ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool NDTScanMatcher::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}
