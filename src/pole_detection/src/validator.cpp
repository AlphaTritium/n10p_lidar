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
      RCLCPP_DEBUG(node_->get_logger(), "✓ Pole %d VALIDATED: r=%.3f, intensity=%.1f",
                  candidate.id, candidate.radius, candidate.avg_intensity);
    } else {
      rejected.push_back(validated_candidate);
      RCLCPP_DEBUG(node_->get_logger(), "✗ Pole %d REJECTED: %s",
                  candidate.id, reject_reason.c_str());
    }
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "Validation: %zu accepted, %zu rejected",
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

bool validator::validateShape(const PoleCandidate& /*candidate*/, const pcl::PointXYZI& /*cluster*/)
{
  return true;
}

}  // namespace pole_detection