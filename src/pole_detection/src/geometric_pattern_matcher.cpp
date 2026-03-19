// Include PCL headers FIRST before anything else
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

// Then ROS headers
#include "geometric_pattern_matcher.hpp"
#include <algorithm>
#include <cmath>

namespace pole_detection
{

GeometricPatternMatcher::GeometricPatternMatcher(
  rclcpp::Node::SharedPtr node, 
  const Config& config)
  : node_(node), config_(config)
{
  debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/debug/geometric_pattern", 10);
  
  RCLCPP_INFO(node_->get_logger(), 
    "GeometricPatternMatcher initialized");
}

PolePatternResult GeometricPatternMatcher::detectPolePattern(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
{
  PolePatternResult result;
  
  // Convert cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  try {
    pcl::fromROSMsg(*cloud_msg, *cloud);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Cloud conversion failed: %s", e.what());
    return result;
  }
  
  if (cloud->empty()) {
    return result;
  }
  
  // Stage 1: Clustering
  auto candidates = extractClusterCentroids(cloud);
  result.num_candidates = static_cast<int>(candidates.size());
  
  if (candidates.size() < 3) {
    result.debug_message = "Too few candidates";
    return result;
  }
  
  // Stage 2: Line fitting (least squares)
  Eigen::Vector3f line_point, line_direction;
  std::vector<int> inlier_indices;
  
  if (!fitLineLeastSquares(candidates, line_point, line_direction, inlier_indices)) {
    result.debug_message = "Line fitting failed";
    return result;
  }
  
  result.inlier_count = static_cast<int>(inlier_indices.size());
  
  // Stage 3: Project to line
  std::vector<geometry_msgs::msg::Point> inlier_points;
  for (int idx : inlier_indices) {
    inlier_points.push_back(candidates[idx]);
  }
  
  auto positions_1d = projectPointsToLine(inlier_points, line_point, line_direction);
  
  // Stage 4: Pattern matching
  double start_offset;
  std::vector<int> pattern_matches;
  
  if (!findEquallySpacedPattern(positions_1d, start_offset, pattern_matches)) {
    result.debug_message = "No pattern found";
    return result;
  }
  
  // Stage 5: Reconstruct
  auto pole_positions = reconstructPolePositions(line_point, line_direction, start_offset);
  
  // Stage 6: Smoothing
  if (config_.use_tracking) {
    pole_positions = applyTemporalSmoothing(pole_positions);
  }
  
  result.success = true;
  result.pole_positions = pole_positions;
  result.debug_message = "Pattern detected";
  
  RCLCPP_INFO(node_->get_logger(), 
    "✓ Pattern: %d poles", config_.num_poles);
  
  publishDebugVisualization(result, cloud_msg->header);
  return result;
}

std::vector<geometry_msgs::msg::Point> GeometricPatternMatcher::extractClusterCentroids(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
{
  std::vector<geometry_msgs::msg::Point> centroids;
  
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);
  
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(config_.cluster_tolerance);
  ec.setMinClusterSize(config_.cluster_min_size);
  ec.setMaxClusterSize(20);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  
  for (const auto& indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZI> cluster;
    for (int idx : indices.indices) {
      cluster.push_back(cloud->points[idx]);
    }
    
    pcl::PointXYZI centroid_pt;
    pcl::computeCentroid(cluster, centroid_pt);
    
    geometry_msgs::msg::Point centroid;
    centroid.x = centroid_pt.x;
    centroid.y = centroid_pt.y;
    centroid.z = 0.0;
    
    centroids.push_back(centroid);
  }
  
  return centroids;
}

bool GeometricPatternMatcher::fitLineLeastSquares(
  const std::vector<geometry_msgs::msg::Point>& candidates,
  Eigen::Vector3f& line_point,
  Eigen::Vector3f& line_direction,
  std::vector<int>& inlier_indices)
{
  if (candidates.size() < 2) return false;
  
  // Centroid
  double sum_x = 0, sum_y = 0;
  for (const auto& pt : candidates) {
    sum_x += pt.x;
    sum_y += pt.y;
  }
  double mean_x = sum_x / candidates.size();
  double mean_y = sum_y / candidates.size();
  
  line_point << mean_x, mean_y, 0.0;
  
  // Covariance
  double cov_xx = 0, cov_xy = 0, cov_yy = 0;
  for (const auto& pt : candidates) {
    double dx = pt.x - mean_x;
    double dy = pt.y - mean_y;
    cov_xx += dx * dx;
    cov_xy += dx * dy;
    cov_yy += dy * dy;
  }
  
  // Principal eigenvector
  double trace = cov_xx + cov_yy;
  double det = cov_xx * cov_yy - cov_xy * cov_xy;
  double disc = trace * trace - 4 * det;
  
  if (disc < 0) return false;
  
  double lambda = (trace + std::sqrt(disc)) / 2.0;
  double dir_x = cov_xy;
  double dir_y = lambda - cov_xx;
  
  double norm = std::hypot(dir_x, dir_y);
  if (norm < 1e-6) {
    dir_x = 1.0; dir_y = 0.0;
  } else {
    dir_x /= norm;
    dir_y /= norm;
  }
  
  line_direction << dir_x, dir_y, 0.0;
  
  // Inliers
  for (size_t i = 0; i < candidates.size(); i++) {
    double dx = candidates[i].x - mean_x;
    double dy = candidates[i].y - mean_y;
    double dist = std::abs(-dir_y * dx + dir_x * dy);
    
    if (dist <= config_.ransac_distance_threshold) {
      inlier_indices.push_back(static_cast<int>(i));
    }
  }
  
  return inlier_indices.size() >= 3;
}

std::vector<double> GeometricPatternMatcher::projectPointsToLine(
  const std::vector<geometry_msgs::msg::Point>& points,
  const Eigen::Vector3f& line_point,
  const Eigen::Vector3f& line_direction)
{
  std::vector<double> t_vals;
  for (const auto& pt : points) {
    Eigen::Vector3f p(pt.x, pt.y, pt.z);
    double t = (p - line_point).dot(line_direction);
    t_vals.push_back(t);
  }
  std::sort(t_vals.begin(), t_vals.end());
  return t_vals;
}

bool GeometricPatternMatcher::findEquallySpacedPattern(
  const std::vector<double>& positions_1d,
  double& start_offset,
  std::vector<int>& matched_indices)
{
  if (positions_1d.empty()) return false;
  
  int best_count = 0;
  double best_start = 0.0;
  
  for (size_t i = 0; i < positions_1d.size(); i++) {
    double start = positions_1d[i];
    int count = 0;
    
    for (int k = 0; k < config_.num_poles; k++) {
      double expected = start + k * config_.expected_spacing;
      
      for (double pos : positions_1d) {
        if (std::abs(pos - expected) <= config_.position_tolerance) {
          count++;
          break;
        }
      }
    }
    
    if (count > best_count) {
      best_count = count;
      best_start = start;
    }
  }
  
  if (best_count >= 4) {
    start_offset = best_start;
    matched_indices.resize(best_count);
    return true;
  }
  
  return false;
}

std::vector<geometry_msgs::msg::Point> GeometricPatternMatcher::reconstructPolePositions(
  const Eigen::Vector3f& line_point,
  const Eigen::Vector3f& line_direction,
  double start_offset)
{
  std::vector<geometry_msgs::msg::Point> positions;
  for (int k = 0; k < config_.num_poles; k++) {
    double t = start_offset + k * config_.expected_spacing;
    Eigen::Vector3f p = line_point + t * line_direction;
    
    geometry_msgs::msg::Point pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = 0.05;
    positions.push_back(pt);
  }
  return positions;
}

std::vector<geometry_msgs::msg::Point> GeometricPatternMatcher::applyTemporalSmoothing(
  const std::vector<geometry_msgs::msg::Point>& new_positions)
{
  if (tracked_positions.empty()) {
    tracked_positions = new_positions;
    track_ages.assign(config_.num_poles, 1);
    return new_positions;
  }
  
  std::vector<geometry_msgs::msg::Point> smoothed;
  for (size_t i = 0; i < new_positions.size(); i++) {
    geometry_msgs::msg::Point pt;
    if (i < tracked_positions.size()) {
      pt.x = config_.smoothing_alpha * new_positions[i].x + 
             (1.0 - config_.smoothing_alpha) * tracked_positions[i].x;
      pt.y = config_.smoothing_alpha * new_positions[i].y + 
             (1.0 - config_.smoothing_alpha) * tracked_positions[i].y;
      pt.z = 0.05;
      track_ages[i]++;
    } else {
      pt = new_positions[i];
      track_ages.push_back(1);
    }
    smoothed.push_back(pt);
  }
  
  tracked_positions = smoothed;
  return smoothed;
}

void GeometricPatternMatcher::publishDebugVisualization(
  const PolePatternResult& result,
  const std_msgs::msg::Header& header)
{
  if (!debug_pub_ || !result.success) return;
  
  visualization_msgs::msg::MarkerArray markers;
  
  // Line
  visualization_msgs::msg::Marker line;
  line.header = header;
  line.ns = "pattern_line";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.points.push_back(result.pole_positions.front());
  line.points.push_back(result.pole_positions.back());
  line.scale.x = 0.01;
  line.color.g = 1.0;
  line.color.a = 0.5;
  line.lifetime = rclcpp::Duration::from_seconds(0.5);
  markers.markers.push_back(line);
  
  // Poles and labels
  for (size_t i = 0; i < result.pole_positions.size(); i++) {
    visualization_msgs::msg::Marker sphere;
    sphere.header = header;
    sphere.ns = "detected_poles";
    sphere.id = static_cast<int>(i) + 1;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.pose.position = result.pole_positions[i];
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05;
    sphere.color.b = 1.0;
    sphere.color.a = 0.8;
    sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
    markers.markers.push_back(sphere);
    
    visualization_msgs::msg::Marker text;
    text.header = header;
    text.ns = "pole_labels";
    text.id = static_cast<int>(i) + 100;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.pose.position = result.pole_positions[i];
    text.pose.position.z += 0.15;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.1;
    text.text = "P" + std::to_string(i);
    text.lifetime = rclcpp::Duration::from_seconds(0.5);
    markers.markers.push_back(text);
  }
  
  debug_pub_->publish(markers);
}

}  // namespace pole_detection