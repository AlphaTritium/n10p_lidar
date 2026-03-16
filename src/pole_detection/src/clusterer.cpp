#include "clusterer.hpp"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

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
    
    // Convert each cluster to a PoleCandidate
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
      
      // Compute centroid
      pcl::PointXYZI centroid_3d;
      pcl::computeCentroid(*cluster_cloud, centroid_3d);
      
      // Estimate radius (average distance from centroid)
      double sum_dist = 0.0;
      double max_dist = 0.0;
      double intensity_sum = 0.0;
      
      for (const auto& pt : cluster_cloud->points) {
        double dx = pt.x - centroid_3d.x;
        double dy = pt.y - centroid_3d.y;
        double dist = std::hypot(dx, dy);
        sum_dist += dist;
        if (dist > max_dist) max_dist = dist;
        intensity_sum += pt.intensity;
      }
      
      double avg_dist = cluster_cloud->empty() ? 0.0 : sum_dist / cluster_cloud->size();
      double avg_intensity = cluster_cloud->empty() ? 0.0 : intensity_sum / cluster_cloud->size();
      
      // Create candidate
      PoleCandidate candidate;
      candidate.id = candidate_id++;
      candidate.centroid.x = centroid_3d.x;
      candidate.centroid.y = centroid_3d.y;
      candidate.centroid.z = centroid_3d.z;
      candidate.radius = avg_dist * 1.15;  // Scale factor to estimate true radius
      candidate.point_count = cluster_cloud->size();
      candidate.avg_intensity = avg_intensity;
      candidate.timestamp = node_->now();
      candidate.rejection_reason = "";  // Not rejected yet
      
      candidates.push_back(candidate);
      
      RCLCPP_DEBUG(node_->get_logger(),
        "Cluster %d: pts=%d, r=%.3f, pos=(%.3f, %.3f), intensity=%.1f",
        candidate.id, candidate.point_count, candidate.radius,
        candidate.centroid.x, candidate.centroid.y, candidate.avg_intensity);
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

void Clusterer::publishDebugMarkers(
  const std::vector<PoleCandidate>& candidates,
  const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto& candidate = candidates[i];
    
    // Sphere marker at centroid
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header = header;
    sphere_marker.ns = "clusters";
    sphere_marker.id = i * 2;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.pose.position = candidate.centroid;
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = candidate.radius * 2.0;
    sphere_marker.scale.y = candidate.radius * 2.0;
    sphere_marker.scale.z = candidate.radius * 2.0;
    sphere_marker.color.r = 1.0;  // Red for raw clusters
    sphere_marker.color.g = 0.5;
    sphere_marker.color.b = 0.0;
    sphere_marker.color.a = 0.5;
    sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    // Text label
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
    text_marker.scale.z = 0.1;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = "C" + std::to_string(candidate.id) + 
                      "\npts:" + std::to_string(candidate.point_count) +
                      "\nr:" + std::to_string(static_cast<int>(candidate.radius * 1000)) + "mm";
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    marker_array.markers.push_back(sphere_marker);
    marker_array.markers.push_back(text_marker);
  }
  
  debug_pub_->publish(marker_array);
}

}  // namespace pole_detection