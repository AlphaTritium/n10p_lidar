#include "pattern_matcher.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace pole_detection
{

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
  
  if (poles.size() < 2) {
    result.matches = 0;
    result.total_pairs = 0;
    result.match_ratio = 0.0;
    return result;
  }
  
  // Compute all pairwise distances
  std::map<std::pair<int, int>, double> distances;
  for (size_t i = 0; i < poles.size(); ++i) {
    for (size_t j = i + 1; j < poles.size(); ++j) {
      double dx = poles[i].position.x - poles[j].position.x;
      double dy = poles[i].position.y - poles[j].position.y;
      double dist = std::hypot(dx, dy);
      distances[std::make_pair(poles[i].track_id, poles[j].track_id)] = dist;
      result.total_pairs++;
    }
  }
  
  // Check which distances match expected pattern
  result.matches = 0;
  for (const auto& pair_dist : distances) {
    double measured = pair_dist.second;
    bool matched = false;
    
    for (const auto& expected : config_.expected_distances) {
      if (std::abs(measured - expected) <= config_.distance_tolerance) {
        matched = true;
        break;
      }
    }
    
    if (matched) {
      result.matches++;
      RCLCPP_DEBUG(node_->get_logger(),
        "✓ Distance match: P%d-P%.3d = %.3fm (matches expected)",
        pair_dist.first.first, pair_dist.first.second, measured);
    } else {
      RCLCPP_DEBUG(node_->get_logger(),
        "✗ Distance mismatch: P%d-P%d = %.3fm (no match)",
        pair_dist.first.first, pair_dist.first.second, measured);
    }
  }
  
  result.match_ratio = result.total_pairs > 0 
    ? static_cast<double>(result.matches) / result.total_pairs 
    : 0.0;
  
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "Pattern match: %.1f%% (%d/%d pairs match expected)",
    result.match_ratio * 100.0, result.matches, result.total_pairs);
  
  // Publish debug info if enabled
  if (config_.publish_debug) {
    publishDistanceMatrix(distances, poles.size());
    publishMatchMarkers(distances, poles);
  }
  
  return result;
}

void PatternMatcher::publishDistanceMatrix(
  const std::map<std::pair<int, int>, double>& distances,
  size_t num_poles)
{
  std_msgs::msg::Float32MultiArray matrix_msg;
  matrix_msg.layout.dim.resize(2);
  matrix_msg.layout.dim[0].label = "pole_i";
  matrix_msg.layout.dim[0].size = num_poles;
  matrix_msg.layout.dim[0].stride = num_poles;
  matrix_msg.layout.dim[1].label = "pole_j";
  matrix_msg.layout.dim[1].size = num_poles;
  matrix_msg.layout.dim[1].stride = 1;
  
  // Fill matrix (symmetric)
  for (size_t i = 0; i < num_poles; ++i) {
    for (size_t j = 0; j < num_poles; ++j) {
      if (i == j) {
        matrix_msg.data.push_back(0.0);
      } else {
        auto key = i < j ? std::make_pair(i, j) : std::make_pair(j, i);
        auto it = distances.find(key);
        matrix_msg.data.push_back(it != distances.end() ? it->second : 0.0);
      }
    }
  }
  
  matrix_pub_->publish(matrix_msg);
}

void PatternMatcher::publishMatchMarkers(
  const std::map<std::pair<int, int>, double>& distances,
  const std::vector<TrackedPole>& poles)
{
  visualization_msgs::msg::MarkerArray markers;
  
  // Draw lines between poles with distance labels
  for (const auto& pair_dist : distances) {
    int id1 = pair_dist.first.first;
    int id2 = pair_dist.first.second;
    double dist = pair_dist.second;
    
    // Find pole positions
    geometry_msgs::msg::Point p1, p2;
    bool found1 = false, found2 = false;
    
    for (const auto& pole : poles) {
      if (pole.track_id == id1) {
        p1.x = pole.position.x;
        p1.y = pole.position.y;
        p1.z = pole.position.z;
        found1 = true;
      }
      if (pole.track_id == id2) {
        p2.x = pole.position.x;
        p2.y = pole.position.y;
        p2.z = pole.position.z;
        found2 = true;
      }
    }
    
    if (!found1 || !found2) continue;
    
    // Check if this distance matches expected pattern
    bool is_match = false;
    for (const auto& expected : config_.expected_distances) {
      if (std::abs(dist - expected) <= config_.distance_tolerance) {
        is_match = true;
        break;
      }
    }
    
    // Line marker
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "laser_link";
    line_marker.ns = "distance_lines";
    line_marker.id = markers.markers.size();
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    line_marker.scale.x = 0.015;
    line_marker.color.r = is_match ? 0.0 : 1.0;  // Green if match, red if not
    line_marker.color.g = is_match ? 1.0 : 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 0.8;
    line_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    // Distance text label (midpoint)
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "laser_link";
    text_marker.ns = "distance_labels";
    text_marker.id = markers.markers.size();
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = (p1.x + p2.x) / 2.0;
    text_marker.pose.position.y = (p1.y + p2.y) / 2.0;
    text_marker.pose.position.z = (p1.z + p2.z) / 2.0 + 0.15;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.1;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = std::to_string(static_cast<int>(dist * 1000)) + "mm";
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    markers.markers.push_back(line_marker);
    markers.markers.push_back(text_marker);
  }
  
  matches_pub_->publish(markers);
}

}  // namespace pole_detection