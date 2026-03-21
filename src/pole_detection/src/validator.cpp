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
  
  // RANGE-BASED WEIGHTING
  double range_weight = 1.0;
  if (f.range_from_sensor > 0.5) {
    range_weight = 0.7;
  } else if (f.range_from_sensor > 0.3) {
    range_weight = 0.85;
  }
  
  // FEATURE 1: Point count (density)
  max_possible_score += config_.weight_point_count;
  int min_pts = static_cast<int>(config_.min_point_count * range_weight);
  min_pts = std::max(3, min_pts);
  int max_pts = static_cast<int>(config_.max_point_count / range_weight);
  
  if (f.point_count >= min_pts && f.point_count <= max_pts) {
    score += config_.weight_point_count;
  }
  
  // HARD REJECTION: Less than 3 points
  if (f.point_count < 3) {
    RCLCPP_DEBUG(node_->get_logger(),
      "Cluster %d: REJECTED - only %d points (hallucination)",
      f.id, f.point_count);
    return 0.0;
  }
  
  // FEATURE 2: BOUNDING BOX AREA (replaces radius!)
  // Pole cross-section should be ~25-28mm diameter
  // Expected area: π×(0.014)² ≈ 0.0006 m² to π×(0.012)² ≈ 0.00045 m²
  // But we use bounding box, so expect slightly larger
  max_possible_score += config_.weight_radial_width;
  double expected_min_area = 0.0003;  // 3cm × 3mm (sparse returns)
  double expected_max_area = 0.0025;  // 5cm × 5cm (close range)
  
  double bbox_area = f.arc_length;  // We store bbox area in arc_length field
  if (bbox_area >= expected_min_area && bbox_area <= expected_max_area) {
    score += config_.weight_radial_width;
    RCLCPP_DEBUG(node_->get_logger(),
      "Cluster %d: Area OK (%.6fm²)", f.id, bbox_area);
  } else {
    RCLCPP_DEBUG(node_->get_logger(),
      "Cluster %d: Area WRONG (%.6fm², expected %.6f-%.6f)",
      f.id, bbox_area, expected_min_area, expected_max_area);
  }
  
  // FEATURE 3: Radial width (max dimension, not radius!)
  // Pole diameter: 25-28mm, so max dimension should be similar
  max_possible_score += config_.weight_curvature;  // Reuse weight
  double min_width = 0.010;  // 10mm minimum
  double max_width = 0.050;  // 50mm maximum
  
  if (f.radial_width >= min_width && f.radial_width <= max_width) {
    score += config_.weight_curvature;
  }
  
  // FEATURE 4: Range (closer = more reliable)
  max_possible_score += config_.weight_range;
  if (f.range_from_sensor <= config_.max_range) {
    score += config_.weight_range;
  }
  
  // Normalize
  double normalized_score = std::max(0.0, std::min(1.0, score / max_possible_score));
  
  return normalized_score;
}

std::string validator::generateRejectionReason(double score, const ClusterFeatures& f)
{
  if (score >= config_.acceptance_threshold) {
    return "";
  }
  
  std::vector<std::string> reasons;
  
  if (f.point_count < 3) {
    reasons.push_back("Too few points (<3)");
  }
  
  // Check area
  double bbox_area = f.arc_length;
  double expected_min_area = 0.0003;
  double expected_max_area = 0.0025;
  if (bbox_area < expected_min_area || bbox_area > expected_max_area) {
    reasons.push_back("Wrong area (" + std::to_string(static_cast<int>(bbox_area * 1000000)) + "mm²)");
  }
  
  // Check radial width
  if (f.radial_width < 0.010 || f.radial_width > 0.050) {
    reasons.push_back("Wrong width (" + std::to_string(static_cast<int>(f.radial_width * 1000)) + "mm)");
  }
  
  if (f.range_from_sensor > config_.max_range) {
    reasons.push_back("Too far");
  }
  
  if (reasons.empty()) {
    return "Low score";
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