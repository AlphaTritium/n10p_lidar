// ============================================================================
// POLE DETECTION SYSTEM - UNIFIED IMPLEMENTATION
// ============================================================================
// 
// This file implements the complete pole detection pipeline in a single file:
// - Point cloud clustering for pole candidate extraction
// - Multi-feature validation to filter false positives
// - Multi-object tracking with EMA smoothing
// - Pattern matching for pole line detection
// - Action server integration for behavior tree compatibility
//
// KISS Principle: Keep It Simple, Stupid
// All processing logic consolidated for easier maintenance and debugging
//
// ============================================================================

#include "pole_detection_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <algorithm>
#include <cmath>

namespace pole_detection
{

// ============================================================================
// SECTION 1: CLUSTERER IMPLEMENTATION
// Extracts pole candidates from point clouds using Euclidean clustering
// ============================================================================

Clusterer::Clusterer(rclcpp::Node::SharedPtr node, const Config& config)
  : node_(node), config_(config)
{
  if (config_.publish_debug_markers) {
    debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/clusters", 10);
    RCLCPP_INFO(node_->get_logger(), "Clusterer debug publishing ENABLED");
  }
}

std::vector<PoleCandidate> Clusterer::extractClusters(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
  const std_msgs::msg::Header& header)
{
  std::vector<PoleCandidate> candidates;
  
  if (cloud->empty()) {
    return candidates;
  }
  
  // Create KD-tree for clustering
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);
  
  // Euclidean clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(config_.cluster_tolerance);
  ec.setMinClusterSize(config_.cluster_min_size);
  ec.setMaxClusterSize(config_.cluster_max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  
  // Extract features for each cluster
  int cluster_id = 0;
  for (const auto& indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZI> cluster_cloud;
    pcl::copyPointCloud(*cloud, indices, cluster_cloud);
    
    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cluster_cloud, centroid);
    
    // Extract arc features
    geometry_msgs::msg::Point centroid_point;
    centroid_point.x = centroid.x();
    centroid_point.y = centroid.y();
    centroid_point.z = centroid.z();
    
    ClusterFeatures features = extractArcFeatures(cluster_cloud, centroid_point);
    
    // Create candidate
    PoleCandidate candidate;
    candidate.id = cluster_id++;
    candidate.centroid = features.centroid;
    candidate.features = features;
    candidate.rejection_reason.clear();
    
    candidates.push_back(candidate);
  }
  
  // Publish debug markers
  if (config_.publish_debug_markers && debug_pub_) {
    publishDebugMarkers(candidates, header);
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "Extracted %zu clusters", candidates.size());
  return candidates;
}

ClusterFeatures Clusterer::extractArcFeatures(
  const pcl::PointCloud<pcl::PointXYZI>& cluster_points,
  const geometry_msgs::msg::Point& centroid)
{
  ClusterFeatures features;
  features.id = 0;
  features.centroid = centroid;
  features.point_count = cluster_points.size();
  
  // Sort points by angle for arc analysis
  pcl::PointCloud<pcl::PointXYZI> sorted_points = cluster_points;
  sortPointsByAngle(sorted_points, centroid);
  
  // Compute geometric features
  features.arc_length = computeArcLength(sorted_points);
  features.angular_span = computeAngularSpan(sorted_points, centroid);
  features.radial_width = computeRadialWidth(sorted_points);
  features.curvature = fitCircleCurvature(sorted_points);
  
  return features;
}

double Clusterer::computeArcLength(const pcl::PointCloud<pcl::PointXYZI>& sorted_points)
{
  if (sorted_points.size() < 2) return 0.0;
  
  double total_length = 0.0;
  for (size_t i = 1; i < sorted_points.size(); ++i) {
    double dx = sorted_points[i].x - sorted_points[i-1].x;
    double dy = sorted_points[i].y - sorted_points[i-1].y;
    total_length += std::hypot(dx, dy);
  }
  
  return total_length;
}

double Clusterer::computeAngularSpan(
  const pcl::PointCloud<pcl::PointXYZI>& points,
  const geometry_msgs::msg::Point& centroid)
{
  if (points.size() < 2) return 0.0;
  
  std::vector<double> angles;
  for (const auto& point : points) {
    double angle = std::atan2(point.y - centroid.y, point.x - centroid.x);
    angles.push_back(angle);
  }
  
  auto [min_it, max_it] = std::minmax_element(angles.begin(), angles.end());
  double span = *max_it - *min_it;
  
  // Handle wrap-around
  if (span > M_PI) {
    span = 2 * M_PI - span;
  }
  
  return std::abs(span) * 180.0 / M_PI;  // Convert to degrees
}

double Clusterer::computeRadialWidth(const pcl::PointCloud<pcl::PointXYZI>& points)
{
  if (points.empty()) return 0.0;
  
  // Compute centroid
  double cx = 0.0, cy = 0.0;
  for (const auto& point : points) {
    cx += point.x;
    cy += point.y;
  }
  cx /= points.size();
  cy /= points.size();
  
  // Compute radial distances
  std::vector<double> radii;
  for (const auto& point : points) {
    double dx = point.x - cx;
    double dy = point.y - cy;
    radii.push_back(std::hypot(dx, dy));
  }
  
  auto [min_it, max_it] = std::minmax_element(radii.begin(), radii.end());
  return *max_it - *min_it;
}

double Clusterer::fitCircleCurvature(const pcl::PointCloud<pcl::PointXYZI>& points)
{
  if (points.size() < 3) return 0.0;
  
  // Simple curvature estimate using three points
  size_t mid = points.size() / 2;
  double x1 = points[0].x, y1 = points[0].y;
  double x2 = points[mid].x, y2 = points[mid].y;
  double x3 = points.back().x, y3 = points.back().y;
  
  // Compute curvature using circle fitting
  double D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
  if (std::abs(D) < 1e-6) return 0.0;
  
  double cx = ((x1*x1 + y1*y1) * (y2 - y3) + 
              (x2*x2 + y2*y2) * (y3 - y1) + 
              (x3*x3 + y3*y3) * (y1 - y2)) / D;
  double cy = ((x1*x1 + y1*y1) * (x3 - x2) + 
              (x2*x2 + y2*y2) * (x1 - x3) + 
              (x3*x3 + y3*y3) * (x2 - x1)) / D;
  
  double radius = std::hypot(x1 - cx, y1 - cy);
  return (radius > 0) ? 1.0 / radius : 0.0;
}

double Clusterer::computeConvexHullArea(const pcl::PointCloud<pcl::PointXYZI>& points)
{
  // Simplified convex hull area estimation
  if (points.size() < 3) return 0.0;
  
  double area = 0.0;
  size_t n = points.size();
  for (size_t i = 0; i < n; ++i) {
    size_t j = (i + 1) % n;
    area += points[i].x * points[j].y;
    area -= points[j].x * points[i].y;
  }
  
  return std::abs(area) / 2.0;
}

void Clusterer::sortPointsByAngle(
  pcl::PointCloud<pcl::PointXYZI>& points,
  const geometry_msgs::msg::Point& centroid)
{
  std::sort(points.begin(), points.end(),
    [&centroid](const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
      double angle_a = std::atan2(a.y - centroid.y, a.x - centroid.x);
      double angle_b = std::atan2(b.y - centroid.y, b.x - centroid.x);
      return angle_a < angle_b;
    });
}

void Clusterer::publishDebugMarkers(
  const std::vector<PoleCandidate>& candidates,
  const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.ns = "clusters";
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  
  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto& candidate = candidates[i];
    
    // Sphere marker at cluster centroid
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header = header;
    sphere_marker.ns = "clusters";
    sphere_marker.id = i;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.pose.position = candidate.centroid;
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = 0.1;
    sphere_marker.scale.y = 0.1;
    sphere_marker.scale.z = 0.1;
    sphere_marker.color.r = 1.0;
    sphere_marker.color.g = 0.5;
    sphere_marker.color.b = 0.0;
    sphere_marker.color.a = 0.6;
    sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    marker_array.markers.push_back(sphere_marker);
  }
  
  debug_pub_->publish(marker_array);
}

// ============================================================================
// SECTION 2: VALIDATOR IMPLEMENTATION
// Multi-feature validation to filter false positives
// ============================================================================

validator::validator(rclcpp::Node::SharedPtr node, const Config& config)
  : node_(node), config_(config)
{
  if (config_.publish_debug) {
    accepted_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/accepted", 10);
    rejected_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/rejected", 10);
    RCLCPP_INFO(node_->get_logger(), "Validator debug publishing ENABLED");
  }
}

std::vector<PoleCandidate> validator::validate(
  const std::vector<PoleCandidate>& candidates,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
{
  std::vector<PoleCandidate> accepted;
  std::vector<PoleCandidate> rejected;
  
  for (const auto& candidate : candidates) {
    double score = computeLikelihoodScore(candidate.features);
    
    if (score >= config_.acceptance_threshold) {
      accepted.push_back(candidate);
      RCLCPP_DEBUG(node_->get_logger(),
        "Candidate %d ACCEPTED (score: %.2f)", candidate.id, score);
    } else {
      auto rejected_candidate = candidate;
      rejected_candidate.rejection_reason = generateRejectionReason(score, candidate.features);
      rejected.push_back(rejected_candidate);
      RCLCPP_DEBUG(node_->get_logger(),
        "Candidate %d REJECTED (score: %.2f, reason: %s)",
        candidate.id, score, rejected_candidate.rejection_reason.c_str());
    }
  }
  
  // Publish debug markers
  if (config_.publish_debug) {
    publishDebugMarkers(accepted, rejected, node_->now());
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "Accepted %zu/%zu candidates",
    accepted.size(), candidates.size());
  return accepted;
}

double validator::computeLikelihoodScore(const ClusterFeatures& features)
{
  double score = 0.0;
  
  // Angular span score (prefer 15-70 degrees)
  if (features.angular_span >= config_.min_angular_span &&
      features.angular_span <= config_.max_angular_span) {
    score += config_.weight_angular_span;
  }
  
  // Point count score (prefer 6-50 points)
  if (features.point_count >= config_.min_point_count &&
      features.point_count <= config_.max_point_count) {
    score += config_.weight_point_count;
  }
  
  // Radial width score (prefer 5-25mm)
  if (features.radial_width >= config_.min_radial_width &&
      features.radial_width <= config_.max_radial_width) {
    score += config_.weight_radial_width;
  }
  
  // Curvature score (prefer moderate curvature)
  if (features.curvature > 0.1 && features.curvature < 10.0) {
    score += config_.weight_curvature;
  }
  
  // Range score (prefer <0.8m)
  double range = std::hypot(features.centroid.x, features.centroid.y);
  if (range <= config_.max_range) {
    score += config_.weight_range;
  }
  
  return score;
}

std::string validator::generateRejectionReason(
  double score, const ClusterFeatures& features)
{
  std::string reason;
  
  if (features.point_count < config_.min_point_count) {
    reason = "Too few points";
  } else if (features.point_count > config_.max_point_count) {
    reason = "Too many points";
  } else if (features.angular_span < config_.min_angular_span) {
    reason = "Angular span too small";
  } else if (features.angular_span > config_.max_angular_span) {
    reason = "Angular span too large";
  } else if (features.radial_width < config_.min_radial_width) {
    reason = "Radial width too small";
  } else if (features.radial_width > config_.max_radial_width) {
    reason = "Radial width too large";
  } else {
    reason = "Low confidence score";
  }
  
  return reason;
}

void validator::publishDebugMarkers(
  const std::vector<PoleCandidate>& accepted,
  const std::vector<PoleCandidate>& rejected,
  const rclcpp::Time& timestamp)
{
  // Publish accepted markers
  visualization_msgs::msg::MarkerArray accepted_array;
  visualization_msgs::msg::Marker clear_accepted;
  clear_accepted.header.stamp = timestamp;
  clear_accepted.header.frame_id = "base_link";
  clear_accepted.ns = "accepted";
  clear_accepted.action = visualization_msgs::msg::Marker::DELETEALL;
  accepted_array.markers.push_back(clear_accepted);
  
  for (size_t i = 0; i < accepted.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = timestamp;
    marker.header.frame_id = "base_link";
    marker.ns = "accepted";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = accepted[i].centroid;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    accepted_array.markers.push_back(marker);
  }
  
  accepted_pub_->publish(accepted_array);
  
  // Publish rejected markers
  visualization_msgs::msg::MarkerArray rejected_array;
  visualization_msgs::msg::Marker clear_rejected;
  clear_rejected.header.stamp = timestamp;
  clear_rejected.header.frame_id = "base_link";
  clear_rejected.ns = "rejected";
  clear_rejected.action = visualization_msgs::msg::Marker::DELETEALL;
  rejected_array.markers.push_back(clear_rejected);
  
  for (size_t i = 0; i < rejected.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = timestamp;
    marker.header.frame_id = "base_link";
    marker.ns = "rejected";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = rejected[i].centroid;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.6;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    rejected_array.markers.push_back(marker);
  }
  
  rejected_pub_->publish(rejected_array);
}

// ============================================================================
// SECTION 3: PATTERN MATCHER IMPLEMENTATION
// Detects pole line patterns with strict colinearity
// ============================================================================

PatternMatcher::PatternMatcher(rclcpp::Node::SharedPtr node, const Config& config)
  : node_(node), config_(config)
{
  if (config_.publish_debug) {
    matrix_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/debug/distance_matrix", 10);
    matches_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/pattern_matches", 10);
    RCLCPP_INFO(node_->get_logger(), "PatternMatcher debug publishing ENABLED");
  }
}

PatternMatchResult PatternMatcher::match(const std::vector<TrackedPole>& poles)
{
  PatternMatchResult result;
  result.matches = 0;
  result.total_pairs = 0;
  result.match_ratio = 0.0;
  
  if (poles.size() < 2) {
    return result;
  }
  
  // Check colinearity first
  if (config_.require_colinear && !arePolesColinear(poles)) {
    RCLCPP_DEBUG(node_->get_logger(), "Poles not colinear, skipping pattern match");
    return result;
  }
  
  // Sort poles along line
  std::vector<TrackedPole> sorted_poles = sortPolesAlongLine(poles);
  
  // Create distance map
  auto distance_map = createDistanceMap(sorted_poles);
  
  // Count matching pairs
  for (const auto& [pair, distance] : distance_map) {
    result.total_pairs++;
    
    // Check if distance matches expected spacing
    bool matches = false;
    for (double expected : config_.expected_distances) {
      if (std::abs(distance - expected) <= config_.distance_tolerance) {
        matches = true;
        break;
      }
    }
    
    if (matches) {
      result.matches++;
      result.matched_pairs.push_back(pair);
    }
  }
  
  // Compute match ratio
  if (result.total_pairs > 0) {
    result.match_ratio = static_cast<double>(result.matches) / result.total_pairs;
  }
  
  // Publish debug info
  if (config_.publish_debug) {
    publishDistanceMatrix(distance_map, sorted_poles.size());
    publishMatchMarkers(distance_map, sorted_poles);
  }
  
  return result;
}

bool PatternMatcher::arePolesColinear(const std::vector<TrackedPole>& poles) const
{
  if (poles.size() < 3) return true;
  
  // Fit line to first two points
  double x1 = poles[0].position.x, y1 = poles[0].position.y;
  double x2 = poles[1].position.x, y2 = poles[1].position.y;
  
  // Check if all points are within tolerance of line
  for (size_t i = 2; i < poles.size(); ++i) {
    double x0 = poles[i].position.x, y0 = poles[i].position.y;
    
    // Distance from point to line
    double distance = std::abs((y2 - y1) * x0 - (x2 - x1) * y0 + 
                            x2 * y1 - y2 * x1) / 
                   std::hypot(y2 - y1, x2 - x1);
    
    if (distance > config_.colinearity_tolerance) {
      return false;
    }
  }
  
  return true;
}

std::vector<TrackedPole> PatternMatcher::sortPolesAlongLine(
  const std::vector<TrackedPole>& poles) const
{
  std::vector<TrackedPole> sorted = poles;
  
  // Sort by x-coordinate (assuming poles are roughly horizontal)
  std::sort(sorted.begin(), sorted.end(),
    [](const TrackedPole& a, const TrackedPole& b) {
      return a.position.x < b.position.x;
    });
  
  return sorted;
}

std::map<std::pair<int, int>, double> PatternMatcher::createDistanceMap(
  const std::vector<TrackedPole>& poles) const
{
  std::map<std::pair<int, int>, double> distances;
  
  for (size_t i = 0; i < poles.size(); ++i) {
    for (size_t j = i + 1; j < poles.size(); ++j) {
      double dx = poles[j].position.x - poles[i].position.x;
      double dy = poles[j].position.y - poles[i].position.y;
      double distance = std::hypot(dx, dy);
      distances[{poles[i].track_id, poles[j].track_id}] = distance;
    }
  }
  
  return distances;
}

void PatternMatcher::publishDistanceMatrix(
  const std::map<std::pair<int, int>, double>& distances,
  size_t num_poles)
{
  std_msgs::msg::Float32MultiArray matrix_msg;
  matrix_msg.data.resize(num_poles * num_poles, 0.0f);
  
  for (const auto& [pair, distance] : distances) {
    auto [i, j] = pair;
    matrix_msg.data[i * num_poles + j] = static_cast<float>(distance);
    matrix_msg.data[j * num_poles + i] = static_cast<float>(distance);
  }
  
  matrix_pub_->publish(matrix_msg);
}

void PatternMatcher::publishMatchMarkers(
  const std::map<std::pair<int, int>, double>& distances,
  const std::vector<TrackedPole>& poles)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = "base_link";
  clear_marker.header.stamp = node_->now();
  clear_marker.ns = "pattern_matches";
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  
  // Draw lines between matching poles
  int marker_id = 0;
  for (const auto& [pair, distance] : distances) {
    auto [id1, id2] = pair;
    
    // Find poles by track ID
    const TrackedPole* pole1 = nullptr, *pole2 = nullptr;
    for (const auto& pole : poles) {
      if (pole.track_id == id1) pole1 = &pole;
      if (pole.track_id == id2) pole2 = &pole;
    }
    
    if (!pole1 || !pole2) continue;
    
    // Check if distance matches expected spacing
    bool matches = false;
    for (double expected : config_.expected_distances) {
      if (std::abs(distance - expected) <= config_.distance_tolerance) {
        matches = true;
        break;
      }
    }
    
    if (matches) {
      visualization_msgs::msg::Marker line_marker;
      line_marker.header.frame_id = "base_link";
      line_marker.header.stamp = node_->now();
      line_marker.ns = "pattern_matches";
      line_marker.id = marker_id++;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      line_marker.pose.orientation.w = 1.0;
      line_marker.scale.x = 0.02;
      line_marker.color.r = 0.0;
      line_marker.color.g = 0.0;
      line_marker.color.b = 1.0;
      line_marker.color.a = 0.8;
      line_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      line_marker.points.push_back(pole1->position);
      line_marker.points.push_back(pole2->position);
      
      marker_array.markers.push_back(line_marker);
    }
  }
  
  matches_pub_->publish(marker_array);
}

// ============================================================================
// SECTION 4: TRACKER IMPLEMENTATION
// Multi-object tracking with EMA smoothing
// ============================================================================

Tracker::Tracker(rclcpp::Node::SharedPtr node, const Config& config)
  : node_(node), config_(config), next_track_id_(0)
{
  if (config_.publish_debug_tracks) {
    debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/tracks", 10);
    RCLCPP_INFO(node_->get_logger(), "Tracker debug publishing ENABLED");
  }
}

std::vector<TrackedPole> Tracker::update(
  const std::vector<PoleCandidate>& detections,
  const std_msgs::msg::Header& header)
{
  auto current_time = node_->now();
  
  // Mark existing tracks as potentially invisible
  for (auto& track : tracks_) {
    track.markInvisible();
  }
  
  // Associate detections with existing tracks
  std::vector<bool> detection_matched(detections.size(), false);
  
  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& detection = detections[i];
    
    // Find best matching track using distance-based association
    int best_track_idx = -1;
    double best_distance = config_.association_distance;
    
    for (size_t j = 0; j < tracks_.size(); ++j) {
      if (!tracks_[j].is_active) continue;
      
      double dx = detection.centroid.x - tracks_[j].position.x;
      double dy = detection.centroid.y - tracks_[j].position.y;
      double distance = std::hypot(dx, dy);
      
      if (distance < best_distance) {
        best_distance = distance;
        best_track_idx = j;
      }
    }
    
    // Update existing track or create new track
    if (best_track_idx >= 0) {
      double confidence = 1.0;
      if (!detection.rejection_reason.empty()) {
        confidence = 0.5;
      }
      
      // Use EMA smoothing with jump detection
      tracks_[best_track_idx].update(detection.centroid, confidence, current_time, 
                                     config_.confirmation_threshold, 
                                     config_.ema_alpha, 
                                     config_.max_jump_distance);
      detection_matched[i] = true;
      
      RCLCPP_DEBUG(node_->get_logger(),
        "Track %d UPDATED: pos=(%.3f, %.3f), detections=%d",
        tracks_[best_track_idx].track_id,
        tracks_[best_track_idx].position.x,
        tracks_[best_track_idx].position.y,
        tracks_[best_track_idx].detection_count);
    } else {
      // Create new track if under limit
      if (static_cast<int>(tracks_.size()) < config_.max_tracks) {
        double confidence = 1.0;
        if (!detection.rejection_reason.empty()) {
          confidence = 0.5;
        }
        
        TrackedPole new_track(next_track_id_++, detection.centroid, confidence, current_time);
        tracks_.push_back(new_track);
        
        RCLCPP_INFO(node_->get_logger(),
          "NEW Track %d created at (%.3f, %.3f)",
          new_track.track_id, detection.centroid.x, detection.centroid.y);
      }
    }
  }
  
  // Remove stale tracks
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
      [this](const TrackedPole& track) {
        bool stale = track.isStale(config_.max_invisible_frames);
        if (stale) {
          RCLCPP_DEBUG(node_->get_logger(),
            "Track %d REMOVED (invisible for %d frames)",
            track.track_id, track.invisible_count);
        }
        return stale;
      }),
    tracks_.end());
  
  // Publish debug visualization
  if (config_.publish_debug_tracks && debug_pub_) {
    publishDebugMarkers(tracks_, header);
  }
  
  return tracks_;
}

void Tracker::publishDebugMarkers(
  const std::vector<TrackedPole>& tracks,
  const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.ns = "tracked_poles";
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  
  for (size_t i = 0; i < tracks.size(); ++i) {
    const auto& track = tracks[i];
    
    // Track color based on confirmation status
    float r, g, b;
    if (track.is_confirmed) {
      r = 0.0; g = 1.0; b = 0.0;  // Green for confirmed
    } else {
      r = 1.0; g = 1.0; b = 0.0;  // Yellow for tentative
    }
    
    // Sphere marker at track position
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header = header;
    sphere_marker.ns = "tracked_poles";
    sphere_marker.id = i * 4;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.pose.position = track.position;
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = track.avg_features_confidence * 0.15;
    sphere_marker.scale.y = track.avg_features_confidence * 0.15;
    sphere_marker.scale.z = track.avg_features_confidence * 0.15;
    sphere_marker.color.r = r;
    sphere_marker.color.g = g;
    sphere_marker.color.b = b;
    sphere_marker.color.a = 0.8;
    sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    // Track ID and detection count label
    visualization_msgs::msg::Marker id_marker;
    id_marker.header = header;
    id_marker.ns = "track_ids";
    id_marker.id = i * 4 + 1;
    id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    id_marker.action = visualization_msgs::msg::Marker::ADD;
    id_marker.pose.position.x = track.position.x;
    id_marker.pose.position.y = track.position.y;
    id_marker.pose.position.z = track.position.z + 0.3;
    id_marker.pose.orientation.w = 1.0;
    id_marker.scale.z = 0.12;
    id_marker.color.r = 1.0;
    id_marker.color.g = 1.0;
    id_marker.color.b = 1.0;
    id_marker.color.a = 1.0;
    
    char text_buf[128];
    snprintf(text_buf, sizeof(text_buf), "T%d\nDets:%d\nY:%.3f", 
             track.track_id, track.detection_count, track.position.y);
    id_marker.text = text_buf;
    id_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    marker_array.markers.push_back(sphere_marker);
    marker_array.markers.push_back(id_marker);
  }
  
  debug_pub_->publish(marker_array);
}

// ============================================================================
// SECTION 5: POLE DETECTION NODE IMPLEMENTATION
// Main node that orchestrates the complete pole detection pipeline
// ============================================================================

PoleDetectionNode::PoleDetectionNode()
  : Node("pole_detection"), modules_initialized_(false)
{
  // Declare parameters
  declare_parameter("cluster_tolerance", 0.04);
  declare_parameter("cluster_min_size", 3);
  declare_parameter("cluster_max_size", 20);
  declare_parameter("publish_debug_clusters", true);
  
  declare_parameter("min_angular_span", 15.0);
  declare_parameter("max_angular_span", 70.0);
  declare_parameter("min_point_count", 3);
  declare_parameter("max_point_count", 30);
  declare_parameter("publish_debug_validation", true);
  
  declare_parameter("max_tracks", 10);
  declare_parameter("association_distance", 0.2);
  declare_parameter("max_invisible_frames", 30);
  declare_parameter("publish_debug_tracks", true);
  declare_parameter("ema_alpha", 0.3);
  declare_parameter("max_jump_distance", 0.5);
  
  declare_parameter("enable_pattern_matching", true);
  declare_parameter("publish_debug_pattern", true);
  
  // Create subscriber
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lslidar_point_cloud", 10,
    std::bind(&PoleDetectionNode::cloudCallback, this, std::placeholders::_1));
  
  // Create publishers
  objects_pub_ = create_publisher<pole_detection::msg::DetectedObjects>(
    "/detected_objects", 10);
  poles_pub_ = create_publisher<pole_detection::msg::DetectedObjects>(
    "/detected_poles", 10);
  
  // Initialize Action Server
  action_server_ = rclcpp_action::create_server<TrackPoles>(
    this,
    "track_poles",
    std::bind(&PoleDetectionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PoleDetectionNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&PoleDetectionNode::handle_accepted, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(get_logger(), "Pole Detection Node initialized");
  RCLCPP_INFO(get_logger(), "Pipeline: Clusterer ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ Validator ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ Tracker ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ PatternMatcher");
  RCLCPP_INFO(get_logger(), "BT-Ready Action Server Started: /track_poles");
}

void PoleDetectionNode::ensureModulesInitialized()
{
  if (modules_initialized_) return;
  initializeModules();
  modules_initialized_ = true;
  RCLCPP_INFO(get_logger(), "All modules initialized");
}

void PoleDetectionNode::initializeModules()
{
  auto shared_node = shared_from_this();
  
  // Initialize Clusterer
  Clusterer::Config cluster_config;
  cluster_config.cluster_tolerance = get_parameter("cluster_tolerance").as_double();
  cluster_config.cluster_min_size = get_parameter("cluster_min_size").as_int();
  cluster_config.cluster_max_size = get_parameter("cluster_max_size").as_int();
  cluster_config.publish_debug_markers = get_parameter("publish_debug_clusters").as_bool();
  clusterer_ = std::make_unique<Clusterer>(shared_node, cluster_config);
  
  // Initialize Validator
  validator::Config validator_config;
  validator_config.min_angular_span = get_parameter("min_angular_span").as_double();
  validator_config.max_angular_span = get_parameter("max_angular_span").as_double();
  validator_config.min_point_count = get_parameter("min_point_count").as_int();
  validator_config.max_point_count = get_parameter("max_point_count").as_int();
  validator_config.publish_debug = get_parameter("publish_debug_validation").as_bool();
  validator_ = std::make_unique<validator>(shared_node, validator_config);
  
  // Initialize Tracker
  Tracker::Config tracker_config;
  tracker_config.max_tracks = get_parameter("max_tracks").as_int();
  tracker_config.association_distance = get_parameter("association_distance").as_double();
  tracker_config.max_invisible_frames = get_parameter("max_invisible_frames").as_int();
  tracker_config.confirmation_threshold = 3;
  tracker_config.publish_debug_tracks = get_parameter("publish_debug_tracks").as_bool();
  tracker_config.ema_alpha = get_parameter("ema_alpha").as_double();
  tracker_config.max_jump_distance = get_parameter("max_jump_distance").as_double();
  tracker_ = std::make_unique<Tracker>(shared_node, tracker_config);
  
  // Initialize PatternMatcher
  PatternMatcher::Config pattern_config;
  pattern_config.expected_distances = {0.185};  // 185mm spacing
  pattern_config.distance_tolerance = 0.015;    // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â±1.5cm tolerance
  pattern_config.enable_harmonics = false;
  pattern_config.require_colinear = true;
  pattern_config.colinearity_tolerance = 0.02;
  pattern_config.min_poles_for_pattern = 4;
  pattern_config.publish_debug = get_parameter("publish_debug_pattern").as_bool();
  pattern_matcher_ = std::make_unique<PatternMatcher>(shared_node, pattern_config);
  
  RCLCPP_INFO(get_logger(), "===========================================");
  RCLCPP_INFO(get_logger(), "POLE DETECTION CONFIGURATION:");
  RCLCPP_INFO(get_logger(), "  - Cluster tolerance: %.2fm", cluster_config.cluster_tolerance);
  RCLCPP_INFO(get_logger(), "  - Min points/cluster: %d", cluster_config.cluster_min_size);
  RCLCPP_INFO(get_logger(), "  - Track confirmation: %d frames", tracker_config.confirmation_threshold);
  RCLCPP_INFO(get_logger(), "  - EMA alpha: %.2f", tracker_config.ema_alpha);
  RCLCPP_INFO(get_logger(), "  - Max jump distance: %.2fm", tracker_config.max_jump_distance);
  RCLCPP_INFO(get_logger(), "===========================================");
}

void PoleDetectionNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  ensureModulesInitialized();
  
  // Convert raw cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  try {
    pcl::fromROSMsg(*msg, *cloud);
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "Cloud conversion failed: %s", e.what());
    return;
  }
  
  if (cloud->empty()) {
    RCLCPP_DEBUG(get_logger(), "Empty cloud");
    return;
  }
  
  // Stage 1: Clustering
  auto candidates = clusterer_->extractClusters(cloud, msg->header);
  
  // Stage 2: Validation
  auto validated = validator_->validate(candidates, cloud);
  
  // Stage 3: Tracking
  auto tracked = tracker_->update(validated, msg->header);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "Tracking %zu poles", tracked.size());
  
  // Stage 4: Pattern Matching
  if (!tracked.empty() && tracked.size() >= 2) {
    auto match_result = pattern_matcher_->match(tracked);
    
    if (match_result.match_ratio > 0.5) {
      RCLCPP_INFO(get_logger(),
        "ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¦ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã¢â‚¬Å“ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€šÃ‚Â¦ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œ Pattern match: %.1f%% (%d/%d pairs)",
        match_result.match_ratio * 100.0, match_result.matches, match_result.total_pairs);
    }
  }
  
  // Publish results
  if (!tracked.empty()) {
    pole_detection::msg::DetectedObjects poles_msg;
    poles_msg.header = msg->header;
    
    for (const auto& pole : tracked) {
      pole_detection::msg::DetectedObject obj;
      obj.label = "pole_" + std::to_string(pole.track_id);
      obj.x = pole.position.x;
      obj.y = pole.position.y;
      obj.z = 0.0;
      obj.confidence = std::min(1.0, pole.detection_count / 10.0);
      poles_msg.objects.push_back(obj);
    }
    
    poles_pub_->publish(poles_msg);
    objects_pub_->publish(poles_msg);
  }
}

// Action Server Implementation
rclcpp_action::GoalResponse PoleDetectionNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, 
  std::shared_ptr<const TrackPoles::Goal> /*goal*/) {
  RCLCPP_INFO(get_logger(), "Goal Received: Starting pole tracking.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PoleDetectionNode::handle_cancel(
  const std::shared_ptr<GoalHandleTrackPoles> /*goal_handle*/) {
  RCLCPP_INFO(get_logger(), "Cancel Request Received: Preempting current tracking.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PoleDetectionNode::handle_accepted(const std::shared_ptr<GoalHandleTrackPoles> goal_handle) {
  std::thread{std::bind(&PoleDetectionNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void PoleDetectionNode::execute(const std::shared_ptr<GoalHandleTrackPoles> goal_handle) {
  action_active_ = true;
  RCLCPP_INFO(get_logger(), "Starting pole tracking action execution");
  
  std::thread{std::bind(&PoleDetectionNode::feedbackLoop, this, std::placeholders::_1), goal_handle}.detach();
  
  rclcpp::Rate loop_rate(1);
  auto result = std::make_shared<TrackPoles::Result>();
  
  while (rclcpp::ok() && action_active_) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      action_active_ = false;
      RCLCPP_INFO(get_logger(), "Tracking Canceled.");
      return;
    }
    loop_rate.sleep();
  }
  
  result->success = true;
  goal_handle->succeed(result);
  action_active_ = false;
  RCLCPP_INFO(get_logger(), "Tracking Completed Successfully.");
}

void PoleDetectionNode::feedbackLoop(const std::shared_ptr<GoalHandleTrackPoles> goal_handle) {
  rclcpp::Rate loop_rate(10);
  auto feedback = std::make_shared<TrackPoles::Feedback>();
  
  while (rclcpp::ok() && action_active_ && !goal_handle->is_canceling()) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto tracks = tracker_->getTracks();
    
    feedback->detected_poles_count = tracks.size();
    feedback->pole_positions.clear();
    feedback->pole_distances_x.clear();
    feedback->pole_distances_y.clear();
    feedback->pole_confidences.clear();
    
    for (const auto& track : tracks) {
      geometry_msgs::msg::Point point;
      point.x = track.position.x;
      point.y = track.position.y;
      point.z = track.position.z;
      feedback->pole_positions.push_back(point);
      feedback->pole_distances_x.push_back(track.position.x);
      feedback->pole_distances_y.push_back(track.position.y);
      feedback->pole_confidences.push_back(track.avg_features_confidence);
    }
    
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }
}

}  // namespace pole_detection

// ============================================================================
// MAIN APPLICATION ENTRY POINT
// ============================================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pole_detection::PoleDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}