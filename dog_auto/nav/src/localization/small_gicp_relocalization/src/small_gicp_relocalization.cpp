// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <algorithm>
#include <cmath>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity()),
  has_valid_result_(false),
  state_(LocalizationState::WAITING_FOR_SCAN),
  global_search_retry_count_(0)
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("max_iterations", 20);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  // Global search parameters
  this->declare_parameter("global_search_enable", true);
  this->declare_parameter("global_search_min_x", -5.0);
  this->declare_parameter("global_search_max_x", 8.0);
  this->declare_parameter("global_search_min_y", -5.0);
  this->declare_parameter("global_search_max_y", 8.0);
  this->declare_parameter("global_search_step_xy", 1.5);
  this->declare_parameter("global_search_step_yaw_deg", 45.0);
  this->declare_parameter("global_search_quick_iterations", 5);
  this->declare_parameter("global_search_refine_iterations", 20);
  this->declare_parameter("global_search_top_n", 5);
  this->declare_parameter("global_search_min_points", 5000);
  this->declare_parameter("global_search_max_retries", 10);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  this->get_parameter("global_search_enable", global_search_enable_);
  this->get_parameter("global_search_min_x", global_search_min_x_);
  this->get_parameter("global_search_max_x", global_search_max_x_);
  this->get_parameter("global_search_min_y", global_search_min_y_);
  this->get_parameter("global_search_max_y", global_search_max_y_);
  this->get_parameter("global_search_step_xy", global_search_step_xy_);
  this->get_parameter("global_search_step_yaw_deg", global_search_step_yaw_deg_);
  this->get_parameter("global_search_quick_iterations", global_search_quick_iterations_);
  this->get_parameter("global_search_refine_iterations", global_search_refine_iterations_);
  this->get_parameter("global_search_top_n", global_search_top_n_);
  this->get_parameter("global_search_min_points", global_search_min_points_);
  this->get_parameter("global_search_max_retries", global_search_max_retries_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters (used as fallback if not doing global search)
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
    has_valid_result_ = true;
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));

  RCLCPP_INFO(
    this->get_logger(),
    "small_gicp_relocalization started. Global search: %s, "
    "search area: [%.1f,%.1f] x [%.1f,%.1f], step_xy=%.1fm, step_yaw=%.0f°",
    global_search_enable_ ? "enabled" : "disabled",
    global_search_min_x_, global_search_max_x_,
    global_search_min_y_, global_search_max_y_,
    global_search_step_xy_, global_search_step_yaw_deg_);
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  // In WAITING_FOR_SCAN, accumulate more points for global search
  // In GLOBAL_SEARCH, also accumulate (for retry)
  // In TRACKING, accumulate for the periodic registration
  *accumulated_cloud_ += *scan;
}

std::vector<SearchCandidate> SmallGicpRelocalizationNode::generateSearchHypotheses()
{
  std::vector<SearchCandidate> hypotheses;
  const double yaw_step_rad = global_search_step_yaw_deg_ * M_PI / 180.0;

  for (double x = global_search_min_x_; x <= global_search_max_x_ + 1e-6;
       x += global_search_step_xy_)
  {
    for (double y = global_search_min_y_; y <= global_search_max_y_ + 1e-6;
         y += global_search_step_xy_)
    {
      for (double yaw = -M_PI; yaw < M_PI - 1e-6; yaw += yaw_step_rad)
      {
        SearchCandidate c;
        c.pose = Eigen::Isometry3d::Identity();
        c.pose.translation() << x, y, 0.0;
        c.pose.linear() =
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        c.score = 0.0;
        c.converged = false;
        hypotheses.push_back(c);
      }
    }
  }

  return hypotheses;
}

double SmallGicpRelocalizationNode::scoreResult(
  const small_gicp::RegistrationResult & result, bool converged)
{
  if (!converged) {
    return -1.0;
  }
  // Score: more inliers + lower error = better match
  // Normalize: inliers is typically ~hundreds to thousands, error is ~0.01-0.5
  double inlier_score = static_cast<double>(result.num_inliers) / 100.0;
  double error_penalty = result.error * 10.0;
  return inlier_score - error_penalty;
}

SearchCandidate SmallGicpRelocalizationNode::runGicpFromHypothesis(
  const Eigen::Isometry3d & hypothesis, int max_iterations)
{
  SearchCandidate candidate;
  candidate.pose = hypothesis;

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = max_iterations;

  auto result = register_->align(*target_, *source_, *target_tree_, hypothesis);

  candidate.converged = result.converged;
  candidate.score = scoreResult(result, result.converged);

  if (result.converged) {
    candidate.pose = result.T_target_source;
  }

  return candidate;
}

void SmallGicpRelocalizationNode::startGlobalSearch()
{
  RCLCPP_INFO(
    this->get_logger(),
    "=== Starting global localization (retry %d/%d) ===",
    global_search_retry_count_ + 1, global_search_max_retries_);

  // Prepare source point cloud from accumulated data
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || source_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Global search: no source points after downsampling.");
    accumulated_cloud_->clear();
    return;
  }

  // Generate search hypotheses
  auto hypotheses = generateSearchHypotheses();
  RCLCPP_INFO(
    this->get_logger(),
    "Global search: %zu hypotheses (%.1fm grid, %.0f° yaw), source has %zu points",
    hypotheses.size(), global_search_step_xy_, global_search_step_yaw_deg_, source_->size());

  // Phase 1: Quick GICP from each hypothesis
  std::vector<SearchCandidate> results;
  for (auto & h : hypotheses) {
    auto result = runGicpFromHypothesis(h.pose, global_search_quick_iterations_);
    if (result.converged) {
      results.push_back(result);
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Global search phase 1: %zu/%zu hypotheses converged",
    results.size(), hypotheses.size());

  if (results.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Global search: no hypothesis converged. Will retry after accumulating more scans.");
    accumulated_cloud_->clear();
    global_search_retry_count_++;
    return;
  }

  // Sort by score (descending)
  std::sort(results.begin(), results.end());

  // Phase 2: Refine top N candidates with more iterations
  size_t top_n = std::min(static_cast<size_t>(global_search_top_n_), results.size());
  RCLCPP_INFO(this->get_logger(), "Global search phase 2: refining top %zu candidates", top_n);

  std::vector<SearchCandidate> refined;
  for (size_t i = 0; i < top_n; i++) {
    auto refined_result = runGicpFromHypothesis(results[i].pose, global_search_refine_iterations_);
    if (refined_result.converged) {
      refined.push_back(refined_result);
    }
  }

  if (refined.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Global search: all refined hypotheses diverged. Will retry.");
    accumulated_cloud_->clear();
    global_search_retry_count_++;
    return;
  }

  // Sort refined results and pick the best
  std::sort(refined.begin(), refined.end());
  const auto & best = refined[0];

  // Update result with best match
  result_t_ = previous_result_t_ = best.pose;
  has_valid_result_ = true;

  const Eigen::Vector3d t = best.pose.translation();
  const Eigen::Matrix3d R_mat = best.pose.rotation();
  const double yaw = std::atan2(R_mat(1, 0), R_mat(0, 0));

  RCLCPP_INFO(
    this->get_logger(),
    "=== Global localization SUCCESS ===\n"
    "  Pose: [x=%.2f, y=%.2f, yaw=%.1f°]  score=%.1f\n"
    "  Top 3 scores: [%.1f, %.1f, %.1f]\n"
    "  Entering tracking mode.",
    t.x(), t.y(), yaw * 180.0 / M_PI, best.score,
    refined.size() > 0 ? refined[0].score : 0.0,
    refined.size() > 1 ? refined[1].score : 0.0,
    refined.size() > 2 ? refined[2].score : 0.0);

  // Also publish to /initialpose for other nodes (e.g., waypoint navigator)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
    init_pose_msg.header.frame_id = map_frame_;
    init_pose_msg.header.stamp = this->now();
    init_pose_msg.pose.pose.position.x = t.x();
    init_pose_msg.pose.pose.position.y = t.y();
    init_pose_msg.pose.pose.position.z = t.z();
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    init_pose_msg.pose.pose.orientation.x = q.x();
    init_pose_msg.pose.pose.orientation.y = q.y();
    init_pose_msg.pose.pose.orientation.z = q.z();
    init_pose_msg.pose.pose.orientation.w = q.w();
    // Set covariance (0.25m² xy, ~3° yaw)
    init_pose_msg.pose.covariance[0] = 0.25;   // xx
    init_pose_msg.pose.covariance[7] = 0.25;   // yy
    init_pose_msg.pose.covariance[35] = 0.0027; // yaw (~3° in rad²)
    init_pose_pub_->publish(init_pose_msg);
  }
  RCLCPP_INFO(this->get_logger(), "Published global localization result to /initialpose");

  // Clear accumulated cloud for tracking mode
  accumulated_cloud_->clear();

  // Enter tracking mode
  state_ = LocalizationState::TRACKING;
  global_search_retry_count_ = 0;
  last_global_search_time_ = this->now();
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    return;
  }

  switch (state_) {
    case LocalizationState::WAITING_FOR_SCAN: {
      // Wait until we have enough points to do a meaningful global search
      if (accumulated_cloud_->size() < static_cast<size_t>(global_search_min_points_)) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Accumulating points for global search: %zu / %d",
          accumulated_cloud_->size(), global_search_min_points_);
        return;
      }

      if (global_search_enable_) {
        RCLCPP_INFO(
          this->get_logger(),
          "Enough points accumulated (%zu), starting global search.",
          accumulated_cloud_->size());
        state_ = LocalizationState::GLOBAL_SEARCH;
        startGlobalSearch();
      } else {
        // Skip global search, use init_pose and go straight to tracking
        RCLCPP_INFO(this->get_logger(), "Global search disabled, entering tracking mode.");
        state_ = LocalizationState::TRACKING;
        // fall through to tracking
      }
      break;
    }

    case LocalizationState::GLOBAL_SEARCH: {
      // If global search hasn't succeeded yet, retry with new accumulated data
      if (global_search_retry_count_ >= global_search_max_retries_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Global search failed after %d retries. Giving up. "
          "Will use init_pose as fallback and enter tracking mode.",
          global_search_max_retries_);
        // Use init_pose (already set) and enter tracking
        has_valid_result_ = true;
        state_ = LocalizationState::TRACKING;
        accumulated_cloud_->clear();
        return;
      }

      // Check if we have enough new points to retry
      if (accumulated_cloud_->size() >= static_cast<size_t>(global_search_min_points_)) {
        RCLCPP_INFO(
          this->get_logger(),
          "Retrying global search with %zu new points (retry %d/%d)",
          accumulated_cloud_->size(),
          global_search_retry_count_ + 1, global_search_max_retries_);
        startGlobalSearch();
      }
      break;
    }

    case LocalizationState::TRACKING: {
      // Normal tracking: prepare source from accumulated cloud
      source_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *accumulated_cloud_, registered_leaf_size_);

      small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

      source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        source_, small_gicp::KdTreeBuilderOMP(num_threads_));

      if (!source_ || !source_tree_) {
        accumulated_cloud_->clear();
        return;
      }

      // Tracking relaxation: if the cloud is small (< 2000 downsampled points),
      // skip this cycle and keep accumulating — more points = better convergence.
      if (source_->size() < 1000) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Tracking: only %zu downsampled points, deferring registration.", source_->size());
        return;  // keep accumulated_cloud_, add more on next callback
      }

      register_->reduction.num_threads = num_threads_;
      register_->rejector.max_dist_sq = max_dist_sq_;
      register_->optimizer.max_iterations = max_iterations_;

      auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

      if (result.converged) {
        result_t_ = previous_result_t_ = result.T_target_source;
        has_valid_result_ = true;
        accumulated_cloud_->clear();  // only clear on success
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "GICP did not converge in tracking (%zu source pts, %d inliers). "
          "Keeping previous result and accumulating more points.",
          source_->size(), result.num_inliers);
        // Don't clear — accumulate more points and retry next cycle
        // Keep previous_result_t_ unchanged (robot likely hasn't moved far)
      }
      break;
    }
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (!has_valid_result_ || result_t_.matrix().isZero()) {
    return;
  }

  // Don't publish until we have a valid scan timestamp
  if (last_scan_time_.seconds() == 0) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // Use scan time so map→odom TF aligns with odom→base_footprint TF (both use
  // registered_scan timestamp from loam_interface). This avoids TF2 extrapolation
  // errors when Nav2 chains the two transforms in the same lookup call.
  transform_stamped.header.stamp = last_scan_time_;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  // ★狗: 强制 2D 定位 —— 只保留 yaw, 抛弃 pitch/roll, 保持 map->odom 水平
  //   ⚠ 绝不能用 Eigen eulerAngles(0,1,2): 它把首轴(roll)规约到 [0,π],
  //   GICP 6-DOF 结果只要带极小【负】roll(噪声, 每周期随机出现) 就会翻转成
  //   roll=π/pitch=π/yaw±π 的等价解 → 抽出的 yaw 偏 180° → map->odom 随机翻转,
  //   定位跳到镜像位置、看似"不收敛"。改用 atan2(R10,R00) 直接取 yaw, 免疫此翻转。
  const Eigen::Matrix3d R = rotation.toRotationMatrix();
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const Eigen::Quaterniond yaw_only(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  transform_stamped.transform.rotation.x = yaw_only.x();
  transform_stamped.transform.rotation.y = yaw_only.y();
  transform_stamped.transform.rotation.z = yaw_only.z();
  transform_stamped.transform.rotation.w = yaw_only.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  // Manual initial pose overrides global search
  if (state_ == LocalizationState::GLOBAL_SEARCH || state_ == LocalizationState::WAITING_FOR_SCAN) {
    RCLCPP_INFO(this->get_logger(), "Manual initial pose received, skipping global search.");
    state_ = LocalizationState::TRACKING;
    global_search_retry_count_ = 0;
  }

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    has_valid_result_ = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
