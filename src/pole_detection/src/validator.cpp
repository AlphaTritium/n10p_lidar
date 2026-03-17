#include "validator.hpp"
#include <pcl/common/common.h>
#include <algorithm>
#include <cmath>

namespace pole_detection
{

validator::validator(rclcpp::Node::SharedPtr node, const Config& config)
  : node_(node), config_(config)
{
  if (config_.publish_debug) {
    accepted_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/validated_poles", 10);
    rejected_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/rejected_poles", 10);
    RCLCPP_INFO(node_->get_logger(), "validator debug publishing ENABLED");
  }
}

std::vector<PoleCandidate> validator::validate(
  const std::vector<PoleCandidate>& candidates,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
{
  (void)cloud;
  
  std::vector<PoleCandidate> validated;
  std::vector<PoleCandidate> rejected;
  
  for (const auto& candidate : candidates) {
    bool is_valid = true;
    std::string reject_reason;
    
    if (!validateRadius(candidate)) {
      is_valid = false;
      reject_reason = "Radius mismatch: " + std::to_string(candidate.radius) + 
                     "m (expected: " + std::to_string(config_.expected_radius) + 
                     "±" + std::to_string(config_.radius_tolerance) + ")";
    }
    
    if (is_valid && config_.use_intensity_stats) {
      if (candidate.avg_intensity < config_.min_intensity) {
        is_valid = false;
        reject_reason = "Low intensity: " + std::to_string(candidate.avg_intensity) + 
                       " (min: " + std::to_string(config_.min_intensity) + ")";
      }
    }
    
    PoleCandidate validated_candidate = candidate;
    validated_candidate.rejection_reason = reject_reason;
    
    if (is_valid) {
      validated.push_back(validated_candidate);
      RCLCPP_DEBUG(node_->get_logger(), "✓ Pole %d validATED: r=%.3f, intensity=%.1f",
                  candidate.id, candidate.radius, candidate.avg_intensity);
    } else {
      rejected.push_back(validated_candidate);
      RCLCPP_DEBUG(node_->get_logger(), "✗ Pole %d REJECTED: %s",
                  candidate.id, reject_reason.c_str());
    }
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "validation: %zu accepted, %zu rejected",
              validated.size(), rejected.size());
  
  return validated;
}

bool validator::validateRadius(const PoleCandidate& candidate)
{
  double error = std::abs(candidate.radius - config_.expected_radius);
  (void)error;
  return error <= config_.radius_tolerance;
}

bool validator::validateIntensity(const PoleCandidate& candidate, const pcl::PointXYZI& /*cluster*/)
{
  return candidate.avg_intensity >= config_.min_intensity;
}

// ... existing code ...

bool validator::validateShape(const PoleCandidate& /*candidate*/, const pcl::PointXYZI& /*cluster*/)
{
  return true;
}

void validator::publishDebugMarkers(
  const std::vector<PoleCandidate>& accepted,
  const std::vector<PoleCandidate>& rejected,
  const rclcpp::Time& timestamp)
{
  std_msgs::msg::Header header;
  header.stamp = timestamp;
  header.frame_id = "laser_link";
  
  // Publish accepted poles
  if (accepted_pub_) {
    visualization_msgs::msg::MarkerArray accepted_markers;
    
    for (size_t i = 0; i < accepted.size(); ++i) {
      const auto& pole = accepted[i];
      
      // Sphere marker
      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "accepted_poles";
      sphere.id = i * 2;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.pose.position = pole.centroid;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = pole.radius * 2.0 * 3.0;  // Exaggerate for visibility
      sphere.scale.y = pole.radius * 2.0 * 3.0;
      sphere.scale.z = pole.radius * 2.0 * 3.0;
      sphere.color.r = 0.0;
      sphere.color.g = 1.0;  // Green
      sphere.color.b = 0.0;
      sphere.color.a = 0.8;
      sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      // Text label
      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns = "accepted_labels";
      text.id = i * 2 + 1;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position = pole.centroid;
      text.pose.position.z += 0.2;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.1;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      text.text = "P" + std::to_string(pole.id) + 
                 "\nr:" + std::to_string(static_cast<int>(pole.radius * 1000)) + "mm";
      text.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      accepted_markers.markers.push_back(sphere);
      accepted_markers.markers.push_back(text);
    }
    
    accepted_pub_->publish(accepted_markers);
  }
  
  // Publish rejected poles
  if (rejected_pub_) {
    visualization_msgs::msg::MarkerArray rejected_markers;
    
    for (size_t i = 0; i < rejected.size(); ++i) {
      const auto& pole = rejected[i];
      
      // Sphere marker
      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "rejected_poles";
      sphere.id = i * 2;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.pose.position = pole.centroid;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = pole.radius * 2.0 * 3.0;
      sphere.scale.y = pole.radius * 2.0 * 3.0;
      sphere.scale.z = pole.radius * 2.0 * 3.0;
      sphere.color.r = 1.0;  // Red
      sphere.color.g = 1.0;
      sphere.color.b = 0.0;
      sphere.color.a = 0.5;
      sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      // Rejection reason label
      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns = "rejected_labels";
      text.id = i * 2 + 1;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position = pole.centroid;
      text.pose.position.z += 0.25;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.08;
      text.color.r = 1.0;
      text.color.g = 0.5;
      text.color.b = 0.0;
      text.color.a = 1.0;
      text.text = "X P" + std::to_string(pole.id) + 
                 "\n" + pole.rejection_reason.substr(0, 30);
      text.lifetime = rclcpp::Duration::from_seconds(1.0);
      
      rejected_markers.markers.push_back(sphere);
      rejected_markers.markers.push_back(text);
    }
    
    rejected_pub_->publish(rejected_markers);
  }
}

}  // namespace pole_detection
