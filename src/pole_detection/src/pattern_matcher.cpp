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
  
  RCLCPP_INFO(node_->get_logger(), 
    "PatternMatcher initialized: %d poles expected, spacing=%.3fm",
    config_.min_poles_for_pattern, config_.expected_distances[0]);
}

PatternMatchResult PatternMatcher::match(const std::vector<TrackedPole>& poles)
{
  PatternMatchResult result;
  
  if (poles.size() < static_cast<size_t>(config_.min_poles_for_pattern)) {
    result.matches = 0;
    result.total_pairs = 0;
    result.match_ratio = 0.0;
    return result;
  }
  
  // Step 1: Check if all poles are colinear
  if (!arePolesColinear(poles)) {
    RCLCPP_DEBUG(node_->get_logger(),
      "Poles are not colinear (tolerance: %.2fm)", config_.colinearity_tolerance);
    result.matches = 0;
    result.total_pairs = 0;
    result.match_ratio = 0.0;
    return result;
  }
  
  // Step 2: Sort poles along the line
  std::vector<TrackedPole> sorted_poles = sortPolesAlongLine(poles);
  
  if (sorted_poles.size() < static_cast<size_t>(config_.min_poles_for_pattern)) {
    result.matches = 0;
    result.total_pairs = 0;
    result.match_ratio = 0.0;
    return result;
  }
  
  // Step 3: Check consecutive distances (must be ~185mm)
  result.matches = 0;
  result.total_pairs = static_cast<int>(sorted_poles.size()) - 1;
  
  for (size_t i = 0; i < sorted_poles.size() - 1; ++i) {
    double dx = sorted_poles[i].position.x - sorted_poles[i+1].position.x;
    double dy = sorted_poles[i].position.y - sorted_poles[i+1].position.y;
    double distance = std::hypot(dx, dy);
    
    // Check if distance matches expected spacing
    for (const auto& expected : config_.expected_distances) {
      if (std::abs(distance - expected) <= config_.distance_tolerance) {
        result.matches++;
        result.matched_pairs.push_back(std::make_pair(sorted_poles[i].track_id, sorted_poles[i+1].track_id));
        result.matched_harmonics[std::make_pair(sorted_poles[i].track_id, sorted_poles[i+1].track_id)] = 1;
        
        RCLCPP_DEBUG(node_->get_logger(),
          "✓ Consecutive poles P%d-P%d: %.3fm (matches %.3f ±%.3fm)",
          sorted_poles[i].track_id, sorted_poles[i+1].track_id,
          distance, expected, config_.distance_tolerance);
        break;
      }
    }
  }
  
  result.match_ratio = result.total_pairs > 0 
    ? static_cast<double>(result.matches) / result.total_pairs 
    : 0.0;
  
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "STRICT COLINEAR Pattern: %.1f%% (%d/%d consecutive pairs match 185mm)",
    result.match_ratio * 100.0, result.matches, result.total_pairs);
  
  // Publish debug info if enabled
  if (config_.publish_debug) {
    publishDistanceMatrix(createDistanceMap(sorted_poles), sorted_poles.size());
    publishMatchMarkers(createDistanceMap(sorted_poles), sorted_poles);
  }
  
  return result;
}

bool PatternMatcher::arePolesColinear(const std::vector<TrackedPole>& poles) const
{
  if (poles.size() < 2) return false;
  
  // Fit a line using least squares
  double sum_x = 0, sum_y = 0;
  for (const auto& pole : poles) {
    sum_x += pole.position.x;
    sum_y += pole.position.y;
  }
  double mean_x = sum_x / poles.size();
  double mean_y = sum_y / poles.size();
  
  // Compute covariance matrix
  double cov_xx = 0, cov_xy = 0, cov_yy = 0;
  for (const auto& pole : poles) {
    double dx = pole.position.x - mean_x;
    double dy = pole.position.y - mean_y;
    cov_xx += dx * dx;
    cov_xy += dx * dy;
    cov_yy += dy * dy;
  }
  
  // Principal component (line direction)
  double trace = cov_xx + cov_yy;
  double det = cov_xx * cov_yy - cov_xy * cov_xy;
  double disc = trace * trace - 4 * det;
  
  if (disc < 0) return false;
  
  double lambda = (trace + std::sqrt(disc)) / 2.0;
  double dir_x = cov_xy;
  double dir_y = lambda - cov_xx;
  
  double norm = std::hypot(dir_x, dir_y);
  if (norm < 1e-6) return false;
  
  dir_x /= norm;
  dir_y /= norm;
  
  // Check perpendicular distances from line
  int inliers = 0;
  for (const auto& pole : poles) {
    double dx = pole.position.x - mean_x;
    double dy = pole.position.y - mean_y;
    
    // Perpendicular distance from point to line
    double perp_dist = std::abs(-dir_y * dx + dir_x * dy);
    
    if (perp_dist <= config_.colinearity_tolerance) {
      inliers++;
    } else {
      RCLCPP_DEBUG(node_->get_logger(),
        "Pole %d deviates %.3fm from line (max: %.3fm)",
        pole.track_id, perp_dist, config_.colinearity_tolerance);
    }
  }
  
  // Require all poles to be on the line
  bool colinear = (inliers == static_cast<int>(poles.size()));
  
  if (colinear) {
    RCLCPP_DEBUG(node_->get_logger(),
      "✓ All %zu poles are colinear (deviation ≤ %.2fm)",
      poles.size(), config_.colinearity_tolerance);
  }
  
  return colinear;
}

std::vector<TrackedPole> PatternMatcher::sortPolesAlongLine(const std::vector<TrackedPole>& poles) const
{
  if (poles.empty()) return {};
  
  // Compute line direction (same as arePolesColinear)
  double sum_x = 0, sum_y = 0;
  for (const auto& pole : poles) {
    sum_x += pole.position.x;
    sum_y += pole.position.y;
  }
  double mean_x = sum_x / poles.size();
  double mean_y = sum_y / poles.size();
  
  double cov_xx = 0, cov_xy = 0, cov_yy = 0;
  for (const auto& pole : poles) {
    double dx = pole.position.x - mean_x;
    double dy = pole.position.y - mean_y;
    cov_xx += dx * dx;
    cov_xy += dx * dy;
    cov_yy += dy * dy;
  }
  
  double trace = cov_xx + cov_yy;
  double det = cov_xx * cov_yy - cov_xy * cov_xy;
  double disc = trace * trace - 4 * det;
  
  double lambda = (trace + std::sqrt(disc)) / 2.0;
  double dir_x = cov_xy;
  double dir_y = lambda - cov_xx;
  
  double norm = std::hypot(dir_x, dir_y);
  if (norm < 1e-6) return poles;
  
  dir_x /= norm;
  dir_y /= norm;
  
  // Project poles onto line and sort by projection
  std::vector<std::pair<double, TrackedPole>> projected_poles;
  for (const auto& pole : poles) {
    double dx = pole.position.x - mean_x;
    double dy = pole.position.y - mean_y;
    double t = dx * dir_x + dy * dir_y;  // Projection onto line
    projected_poles.push_back(std::make_pair(t, pole));
  }
  
  // Sort by projection parameter t
  std::sort(projected_poles.begin(), projected_poles.end(),
    [](const std::pair<double, TrackedPole>& a, const std::pair<double, TrackedPole>& b) {
      return a.first < b.first;
    });
  
  // Extract sorted poles
  std::vector<TrackedPole> sorted;
  for (const auto& proj : projected_poles) {
    sorted.push_back(proj.second);
  }
  
  return sorted;
}

std::map<std::pair<int, int>, double> PatternMatcher::createDistanceMap(
  const std::vector<TrackedPole>& poles) const
{
  std::map<std::pair<int, int>, double> distances;
  for (size_t i = 0; i < poles.size(); ++i) {
    for (size_t j = i + 1; j < poles.size(); ++j) {
      double dx = poles[i].position.x - poles[j].position.x;
      double dy = poles[i].position.y - poles[j].position.y;
      double dist = std::hypot(dx, dy);
      distances[std::make_pair(poles[i].track_id, poles[j].track_id)] = dist;
    }
  }
  return distances;
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
        auto key = i < j ? std::make_pair((int)i, (int)j) : std::make_pair((int)j, (int)i);
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
    
    // Check if this distance matches and which harmonic
    bool is_match = false;
    int harmonic = 0;
    for (const auto& expected_base : config_.expected_distances) {
      for (int h = 1; h <= config_.max_harmonic; h++) {
        if (std::abs(dist - expected_base * h) <= config_.distance_tolerance) {
          is_match = true;
          harmonic = h;
          break;
        }
      }
      if (is_match) break;
    }
    
    // Line marker (green if match, red if not)
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
    
    // Distance text label with harmonic info
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
    text_marker.text = std::to_string(static_cast<int>(dist * 1000)) + "mm" +
                      (is_match && harmonic > 1 ? "\n(" + std::to_string(harmonic) + "×)" : "");
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    markers.markers.push_back(line_marker);
    markers.markers.push_back(text_marker);
  }
  
  matches_pub_->publish(markers);
}

}  // namespace pole_detection