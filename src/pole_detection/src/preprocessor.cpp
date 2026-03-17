// src/preprocessor.cpp
#include "preprocessor.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

namespace pole_detection
{

Preprocessor::Preprocessor(rclcpp::Node::SharedPtr node, const Config& config)
  : node_(node), config_(config)
{
  // Only create publisher if debug is explicitly enabled
  // if (config_.publish_debug_cloud) {
  //   debug_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
  //    "/debug/preprocessed_cloud", 10);
    RCLCPP_INFO(node_->get_logger(), "Preprocessor debug publishing ENABLED");
}


pcl::PointCloud<pcl::PointXYZI>::Ptr Preprocessor::process(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input)
{
  if (!input || input->width == 0) {
    return nullptr;
  }
  
  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  try {
    pcl::fromROSMsg(*input, *cloud);
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "Point cloud conversion warning: %s", e.what());
    RCLCPP_WARN(node_->get_logger(), "Attempting to read point cloud without intensity field...");
    
    // Fallback: try to convert as PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud_xyz);
    
    // Convert to PointXYZI with zero intensity
    cloud->header = cloud_xyz->header;
    cloud->width = cloud_xyz->width;
    cloud->height = cloud_xyz->height;
    cloud->is_dense = cloud_xyz->is_dense;
    cloud->points.resize(cloud_xyz->points.size());
    
    for (size_t i = 0; i < cloud_xyz->points.size(); ++i) {
      cloud->points[i].x = cloud_xyz->points[i].x;
      cloud->points[i].y = cloud_xyz->points[i].y;
      cloud->points[i].z = cloud_xyz->points[i].z;
      cloud->points[i].intensity = 0.0;  // Default intensity
    }
  }
  
  // Apply passthrough filters
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  
  for (const auto& pt : cloud->points) {
    double range = std::hypot(pt.x, pt.y);
    
    // Range filter
    if (range < config_.range_min || range > config_.range_max) {
      continue;
    }
    
    // Z-height filter
    if (pt.z < config_.z_min || pt.z > config_.z_max) {
      continue;
    }
    
    // Intensity filter (optional)
    if (config_.use_intensity_filter && pt.intensity < config_.min_intensity) {
      continue;
    }
    
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
      filtered->push_back(pt);
    }
  }
  
  RCLCPP_DEBUG(node_->get_logger(), 
    "Filtered from %zu to %zu points (removed %zu)", 
    cloud->points.size(), filtered->points.size(), 
    cloud->points.size() - filtered->points.size());

  // Voxel grid downsampling
  if (config_.voxel_leaf_size > 0.0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxelize(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(filtered);
    voxel.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size);
    voxel.filter(*voxelize);
    filtered = voxelize;

    RCLCPP_DEBUG(node_->get_logger(), 
      "After voxel filtering: %zu points", filtered->points.size());
  }
  
  // Publish debug if enabled
  // if (config_.publish_debug_cloud && debug_pub_) {
  //   publishDebug(*filtered, input->header);
  //}
  
  return filtered;
}

/**
void Preprocessor::publishDebug(
  const pcl::PointCloud<pcl::PointXYZI>& cloud,
  const std_msgs::msg::Header& header)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = header;
  debug_pub_->publish(msg);
}
*/

}  // namespace pole_detection