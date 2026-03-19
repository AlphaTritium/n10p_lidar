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
    RCLCPP_INFO(node_->get_logger(), "Validator debug publishing ENABLED");
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
    // NEW: Multi-feature likelihood scoring
    double likelihood_score = computeLikelihoodScore(candidate.features);
    
    PoleCandidate scored_candidate = candidate;
    scored_candidate.likelihood_score = likelihood_score;
    
    bool is_valid = likelihood_score >= config_.acceptance_threshold;
    std::string reject_reason = generateRejectionReason(likelihood_score, candidate.features);
    
    scored_candidate.rejection_reason = reject_reason;
    
    if (is_valid) {
      validated.push_back(scored_candidate);
      RCLCPP_DEBUG(node_->get_logger(), 
        "✓ Pole %d ACCEPTED: score=%.2f (ang=%.1f°, pts=%d, width=%.3fm)",
        candidate.id, likelihood_score,
        candidate.features.angular_span, candidate.features.point_count,
        candidate.features.radial_width);
    } else {
      rejected.push_back(scored_candidate);
      RCLCPP_DEBUG(node_->get_logger(), 
        "✗ Pole %d REJECTED: score=%.2f - %s",
        candidate.id, likelihood_score, reject_reason.c_str());
    }
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "Validation: %zu accepted, %zu rejected",
              validated.size(), rejected.size());
  
  // Publish debug markers if enabled
  if (config_.publish_debug && (accepted_pub_ || rejected_pub_)) {
    publishDebugMarkers(validated, rejected, validated.size() > 0 ? validated[0].timestamp : node_->now());
  }
  
  return validated;
}

double validator::computeLikelihoodScore(const ClusterFeatures& f)
{
  double score = 0.0;
  double max_possible_score = 0.0;
  
  // FEATURE 1: Angular span (poles typically 20-60°)
  max_possible_score += config_.weight_angular_span;
  if (f.angular_span >= config_.min_angular_span && 
      f.angular_span <= config_.max_angular_span) {
    score += config_.weight_angular_span;
  } else if (f.angular_span < 10.0 || f.angular_span > 90.0) {
    score -= 0.1;
  }
  
  // FEATURE 2: Point count (density indicator)
  max_possible_score += config_.weight_point_count;
  if (f.point_count >= config_.min_point_count && 
      f.point_count <= config_.max_point_count) {
    score += config_.weight_point_count;
  }
  
  // FEATURE 3: Radial width (thickness proxy, poles are thin)
  max_possible_score += config_.weight_radial_width;
  if (f.radial_width >= config_.min_radial_width && 
      f.radial_width <= config_.max_radial_width) {
    score += config_.weight_radial_width;
  }
  
  // FEATURE 4: Curvature consistency (weak feature)
  max_possible_score += config_.weight_curvature;
  double expected_curvature = 1.0 / config_.expected_radius;
  if (f.curvature_estimate > 0.0 && 
      std::abs(f.curvature_estimate - expected_curvature) < 15.0) {
    score += config_.weight_curvature;
  }
  
  // FEATURE 5: Range consistency (closer = more reliable)
  max_possible_score += config_.weight_range;
  if (f.range_from_sensor <= config_.max_range) {
    score += config_.weight_range;
  }
  
  // Normalize score to 0.0-1.0 range
  double normalized_score = std::max(0.0, std::min(1.0, score / max_possible_score));
  
  return normalized_score;
}

std::string validator::generateRejectionReason(double score, const ClusterFeatures& f)
{
  if (score >= config_.acceptance_threshold) {
    return "";
  }
  
  std::vector<std::string> reasons;
  
  if (f.angular_span < config_.min_angular_span || 
      f.angular_span > config_.max_angular_span) {
    reasons.push_back("Angular span out of range (" + 
                     std::to_string(static_cast<int>(f.angular_span)) + "°)");
  }
  
  if (f.point_count < config_.min_point_count || 
      f.point_count > config_.max_point_count) {
    reasons.push_back("Point count mismatch (" + 
                     std::to_string(f.point_count) + " pts)");
  }
  
  if (f.radial_width < config_.min_radial_width || 
      f.radial_width > config_.max_radial_width) {
    reasons.push_back("Radial width abnormal (" + 
                     std::to_string(static_cast<int>(f.radial_width * 1000)) + "mm)");
  }
  
  if (f.range_from_sensor > config_.max_range) {
    reasons.push_back("Too far (" + 
                     std::to_string(static_cast<int>(f.range_from_sensor * 100)) + "cm)");
  }
  
  if (reasons.empty()) {
    return "Low overall score (" + std::to_string(static_cast<int>(score * 100)) + "%)";
  }
  
  std::string combined;
  for (size_t i = 0; i < reasons.size(); ++i) {
    combined += reasons[i];
    if (i < reasons.size() - 1) combined += "; ";
  }
  
  return combined;
}

bool validator::validateRadius(const PoleCandidate& candidate)
{
  double error = std::abs(candidate.radius - config_.expected_radius);
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
      
      // Green sphere
      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "accepted_poles";
      sphere.id = i * 2;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.pose.position = pole.centroid;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.05;
      sphere.scale.y = 0.05;
      sphere.scale.z = 0.05;
      sphere.color.r = 0.0;
      sphere.color.g = 1.0;
      sphere.color.b = 0.0;
      sphere.color.a = 0.8;
      sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      // Label with score
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
      text.text = "✓P" + std::to_string(pole.id) + 
                 "\nscore:" + std::to_string(static_cast<int>(pole.likelihood_score * 100)) + "%";
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
      
      // Yellow/red sphere
      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "rejected_poles";
      sphere.id = i * 2;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.pose.position = pole.centroid;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.05;
      sphere.scale.y = 0.05;
      sphere.scale.z = 0.05;
      sphere.color.r = 1.0;
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
      text.text = "✗P" + std::to_string(pole.id) + 
                 "\n" + pole.rejection_reason.substr(0, 25);
      text.lifetime = rclcpp::Duration::from_seconds(1.0);
      
      rejected_markers.markers.push_back(sphere);
      rejected_markers.markers.push_back(text);
    }
    
    rejected_pub_->publish(rejected_markers);
  }
}

}  // namespace pole_detection