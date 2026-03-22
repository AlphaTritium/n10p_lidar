#include "clusterer.hpp"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <algorithm>

namespace pole_detection
{

Clusterer::Clusterer(rclcpp::Node::SharedPtr node, const Config& config)
  : node_(node), config_(config)
{
  if (config_.publish_debug_markers) {
    debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/clusters_raw", 10);
    RCLCPP_INFO(node_->get_logger(), "Clusterer debug publishing ENABLED");
  }
}

std::vector<PoleCandidate> Clusterer::extractClusters(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
  const std_msgs::msg::Header& header)
{
  std::vector<PoleCandidate> candidates;
  
  if (!cloud || cloud->empty()) {
    return candidates;
  }
  
  try {
    // Build Kd-tree for efficient neighborhood search
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    
    // Extract Euclidean clusters
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(config_.cluster_tolerance);
    ec.setMinClusterSize(config_.cluster_min_size);
    ec.setMaxClusterSize(config_.cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
    
    RCLCPP_DEBUG(node_->get_logger(), "Extracted %zu raw clusters", cluster_indices.size());
    
    // Convert each cluster to a PoleCandidate with enhanced features
    int candidate_id = 0;
    for (const auto& indices : cluster_indices) {
      if (indices.indices.empty()) continue;
      
      // Extract cluster points
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      for (int idx : indices.indices) {
        if (idx >= 0 && idx < static_cast<int>(cloud->size())) {
          cluster_cloud->push_back(cloud->points[idx]);
        }
      }
      
      // Compute centroid (x,y only for 2D LiDAR)
      pcl::PointXYZI centroid_3d;
      pcl::computeCentroid(*cluster_cloud, centroid_3d);
      
      geometry_msgs::msg::Point centroid;
      centroid.x = centroid_3d.x;
      centroid.y = centroid_3d.y;
      centroid.z = 0.0;  // Ignore z for 2D LiDAR
      
      // NEW: Extract arc-based features
      ClusterFeatures features = extractArcFeatures(*cluster_cloud, centroid);
      features.id = candidate_id;
      features.legacy_radius = 1.0 / (features.curvature_estimate + 1e-6);
      
      // Create candidate
      PoleCandidate candidate;
      candidate.id = candidate_id++;
      candidate.features = features;
      candidate.centroid = centroid;
      candidate.point_count = cluster_cloud->size();
      candidate.avg_intensity = features.avg_intensity;
      candidate.radius = features.legacy_radius;  // For backward compatibility
      candidate.timestamp = node_->now();
      candidate.rejection_reason = "";
      candidate.likelihood_score = 1.0;  // Will be set by validator
      
      RCLCPP_DEBUG(node_->get_logger(),
        "Cluster %d: pts=%d, arc=%.3fm, ang=%.1f°, width=%.3fm, r=%.3fm",
        candidate.id, candidate.point_count, 
        features.arc_length, features.angular_span,
        features.radial_width, features.legacy_radius);
      
      candidates.push_back(candidate);
    }
    
    // Publish debug markers if enabled
    if (config_.publish_debug_markers && debug_pub_) {
      publishDebugMarkers(candidates, header);
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Clustering failed: %s", e.what());
  }
  
  return candidates;
}

ClusterFeatures Clusterer::extractArcFeatures(
  const pcl::PointCloud<pcl::PointXYZI>& cluster_points,
  const geometry_msgs::msg::Point& centroid)
{
  ClusterFeatures features;
  
  // 1. Range from sensor
  features.range_from_sensor = std::hypot(centroid.x, centroid.y);
  
  // 2. Point count
  features.point_count = cluster_points.size();
  
  // 3. Bounding box dimensions (AREA-BASED, NO RADIUS!)
  double min_x = 1e6, max_x = -1e6;
  double min_y = 1e6, max_y = -1e6;
  for (const auto& pt : cluster_points) {
    min_x = std::min(min_x, (double)pt.x);
    max_x = std::max(max_x, (double)pt.x);
    min_y = std::min(min_y, (double)pt.y);
    max_y = std::max(max_y, (double)pt.y);
  }
  
  double extent_x = max_x - min_x;
  double extent_y = max_y - min_y;
  features.radial_width = std::max(extent_x, extent_y);  // Max dimension (not radius!)
  features.arc_length = extent_x * extent_y;  // Bounding box AREA (m²)
  features.angular_span = 0.0;  // Not used
  
  // 4. Convex hull area (more accurate size metric)
  if (cluster_points.size() >= 3) {
    features.curvature_estimate = computeConvexHullArea(cluster_points);
  } else {
    features.curvature_estimate = 0.0;
  }
  
  // 5. Intensity
  double sum_intensity = 0.0;
  for (const auto& pt : cluster_points) {
    sum_intensity += pt.intensity;
  }
  features.avg_intensity = cluster_points.empty() ? 0.0 : sum_intensity / cluster_points.size();
  features.intensity_variance = 0.0;
  
  // 6. Legacy fields - ZEROED (NO RADIUS!)
  features.legacy_radius = 0.0;  // NOT USED!
  
  RCLCPP_DEBUG(node_->get_logger(), 
    "Cluster %d: pts=%d, bbox_area=%.6fm², convex_area=%.6fm², width=%.3fm",
    features.id, features.point_count, 
    features.arc_length, features.curvature_estimate, features.radial_width);
  
  return features;
}

// ... existing code ...

double Clusterer::computeConvexHullArea(const pcl::PointCloud<pcl::PointXYZI>& points)
{
  if (points.size() < 3) return 0.0;
  
  // Compute centroid
  pcl::PointXYZI centroid_3d;
  pcl::computeCentroid(points, centroid_3d);
  
  geometry_msgs::msg::Point centroid;
  centroid.x = centroid_3d.x;
  centroid.y = centroid_3d.y;
  centroid.z = 0.0;
  
  // Sort points angularly
  pcl::PointCloud<pcl::PointXYZI> sorted_points = points;
  sortPointsByAngle(sorted_points, centroid);
  
  // Shoelace formula for polygon area
  double area = 0.0;
  for (size_t i = 0; i < sorted_points.size(); ++i) {
    size_t j = (i + 1) % sorted_points.size();
    area += sorted_points[i].x * sorted_points[j].y;
    area -= sorted_points[j].x * sorted_points[i].y;
  }
  
  return std::abs(area) / 2.0;
}


void Clusterer::sortPointsByAngle(
  pcl::PointCloud<pcl::PointXYZI>& points,
  const geometry_msgs::msg::Point& centroid)
{
  std::sort(points.begin(), points.end(),
    [centroid](const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
      double angle_a = atan2(a.y - centroid.y, a.x - centroid.x);
      double angle_b = atan2(b.y - centroid.y, b.x - centroid.x);
      return angle_a < angle_b;
    });
}

double Clusterer::computeArcLength(const pcl::PointCloud<pcl::PointXYZI>& sorted_points)
{
  if (sorted_points.size() < 2) return 0.0;
  
  double total_length = 0.0;
  for (size_t i = 0; i < sorted_points.size() - 1; ++i) {
    double dx = sorted_points[i+1].x - sorted_points[i].x;
    double dy = sorted_points[i+1].y - sorted_points[i].y;
    total_length += std::hypot(dx, dy);
  }
  
  return total_length;
}

double Clusterer::computeAngularSpan(
  const pcl::PointCloud<pcl::PointXYZI>& points,
  const geometry_msgs::msg::Point& centroid)
{
  if (points.empty()) return 0.0;
  
  double min_angle = M_PI;
  double max_angle = -M_PI;
  
  for (const auto& pt : points) {
    double angle = atan2(pt.y - centroid.y, pt.x - centroid.x);
    min_angle = std::min(min_angle, angle);
    max_angle = std::max(max_angle, angle);
  }
  
  double span_rad = max_angle - min_angle;
  return span_rad * (180.0 / M_PI);  // Convert to degrees
}

double Clusterer::computeRadialWidth(const pcl::PointCloud<pcl::PointXYZI>& points)
{
  if (points.empty()) return 0.0;
  
  double min_range = 1e6;
  double max_range = 0.0;
  
  for (const auto& pt : points) {
    double range = std::hypot(pt.x, pt.y);
    min_range = std::min(min_range, range);
    max_range = std::max(max_range, range);
  }
  
  return max_range - min_range;
}

double Clusterer::fitCircleCurvature(const pcl::PointCloud<pcl::PointXYZI>& points)
{
  // Simple circle fit using least squares
  // For 2D LiDAR arcs, this gives better radius estimate than centroid method
  
  if (points.size() < 3) return 0.0;
  
  // Compute moments
  double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0;
  for (const auto& pt : points) {
    sum_x += pt.x;
    sum_y += pt.y;
    sum_x2 += pt.x * pt.x;
    sum_y2 += pt.y * pt.y;
  }
  
  double n = points.size();
  double Sxx = sum_x2 - sum_x * sum_x / n;
  double Syy = sum_y2 - sum_y * sum_y / n;
  
  // Estimate radius from variance (approximate for circular arcs)
  double variance = (Sxx + Syy) / n;
  double radius_est = std::sqrt(variance);
  
  return radius_est > 0.001 ? 1.0 / radius_est : 0.0;
}

void Clusterer::publishDebugMarkers(
  const std::vector<PoleCandidate>& candidates,
  const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto& candidate = candidates[i];
    const auto& f = candidate.features;
    
    // Sphere marker at centroid
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header = header;
    sphere_marker.ns = "clusters";
    sphere_marker.id = i * 2;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.pose.position = candidate.centroid;
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = f.radial_width * 10.0;  // Exaggerate radial width
    sphere_marker.scale.y = f.radial_width * 10.0;
    sphere_marker.scale.z = f.radial_width * 10.0;
    sphere_marker.color.r = 1.0;  // Orange for raw clusters
    sphere_marker.color.g = 0.5;
    sphere_marker.color.b = 0.0;
    sphere_marker.color.a = 0.5;
    sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    // Text label with arc features
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = header;
    text_marker.ns = "cluster_labels";
    text_marker.id = i * 2 + 1;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = candidate.centroid.x;
    text_marker.pose.position.y = candidate.centroid.y;
    text_marker.pose.position.z = candidate.centroid.z + 0.2;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.08;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = "C" + std::to_string(candidate.id) + 
                      "\npts:" + std::to_string(candidate.point_count) +
                      "\narc:" + std::to_string(static_cast<int>(f.arc_length * 1000)) + "mm" +
                      "\nang:" + std::to_string(static_cast<int>(f.angular_span)) + "°";
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    marker_array.markers.push_back(sphere_marker);
    marker_array.markers.push_back(text_marker);
  }
  
  debug_pub_->publish(marker_array);
}

}  // namespace pole_detection