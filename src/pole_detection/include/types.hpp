// include/types.hpp
#ifndef POLE_DETECTION__TYPES_HPP_
#define POLE_DETECTION__TYPES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>

namespace pole_detection
{

// Forward declaration not needed since we're defining ClusterFeatures first

struct ClusterFeatures
{
  // Basic properties
  int id;
  geometry_msgs::msg::Point centroid;
  int point_count;
  
  // SIZE FEATURES (NO RADIUS!)
  double arc_length;           // NOW: Bounding box AREA (m²)
  double angular_span;         // UNUSED (kept for compatibility)
  double radial_width;         // MAX DIMENSION (not radius!)
  double curvature;   // NOW: Convex hull AREA (m²)
  
  // Intensity
  double avg_intensity;
  double intensity_variance;
  
  // Range
  double range_from_sensor;
  
  // Legacy - DEPRECATED, ALWAYS ZERO
  double legacy_radius;        // NOT USED - always 0.0
  
  ClusterFeatures()
    : id(0)
    , centroid(geometry_msgs::msg::Point())
    , point_count(0)
    , arc_length(0.0)          // Bbox area
    , angular_span(0.0)
    , radial_width(0.0)        // Max dimension
    , curvature(0.0)  // Convex hull area
    , avg_intensity(0.0)
    , intensity_variance(0.0)
    , range_from_sensor(0.0)
    , legacy_radius(0.0)       // ZERO!
  {}
};

struct PoleCandidate
{
  int id;
  ClusterFeatures features;
  geometry_msgs::msg::Point centroid;
  int point_count;
  double avg_intensity;
  double radius;
  double likelihood_score;
  std::string rejection_reason;
  rclcpp::Time timestamp;
  
  bool is_valid() const { return rejection_reason.empty(); }
};

struct TrackedPole
{
  int track_id;
  geometry_msgs::msg::Point position;
  double avg_features_confidence;
  int detection_count;
  int invisible_count;
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  bool is_confirmed;
  bool is_active;
  
  TrackedPole(int id, const geometry_msgs::msg::Point& pos, double confidence, rclcpp::Time stamp)
    : track_id(id), position(pos), avg_features_confidence(confidence), detection_count(1),
      invisible_count(0), first_seen(stamp), last_seen(stamp),
      is_confirmed(false), is_active(true) {}
  
  void update(const geometry_msgs::msg::Point& pos, double confidence, rclcpp::Time stamp, 
             int confirmation_threshold = 5, double ema_alpha = 0.3, double max_jump_distance = 0.5) {
    // Smart EMA with jump detection from rc2026_head_finder
    double jump_dist = std::hypot(pos.x - position.x, pos.y - position.y);
    
    if (jump_dist > max_jump_distance) {
      // Immediate reset on large jumps (target switch detection)
      position.x = pos.x;
      position.y = pos.y;
      RCLCPP_INFO(rclcpp::get_logger("pole_detection"), 
          "Jump detected (%.3fm) - resetting track %d", jump_dist, track_id);
    } else {
      // Normal EMA smoothing with configurable alpha
      position.x = ema_alpha * pos.x + (1.0 - ema_alpha) * position.x;
      position.y = ema_alpha * pos.y + (1.0 - ema_alpha) * position.y;
    }
    
    position.z = 0.05;
    avg_features_confidence = 0.9 * avg_features_confidence + 0.1 * confidence;
    last_seen = stamp;
    detection_count++;
    invisible_count = 0;
    if (detection_count >= confirmation_threshold) is_confirmed = true;
  }
  
  void markInvisible() { invisible_count++; }
  bool isStale(int max_invisible = 20) const { return invisible_count > max_invisible; }
};

struct ClusterDebugInfo
{
  int cluster_id;
  int point_count;
  double arc_length;
  double angular_span;
  double radial_width;
  double likelihood_score;
  geometry_msgs::msg::Point centroid;
  bool was_accepted;
  std::string reason;
};

struct PatternMatchResult
{
  int matches;                    // Number of matching pole pairs
  int total_pairs;                // Total possible pairs
  double match_ratio;             // matches / total_pairs
  std::vector<std::pair<int, int>> matched_pairs;  // Indices of matching poles
  
  PatternMatchResult()
    : matches(0), total_pairs(0), match_ratio(0.0)
  {}
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__TYPES_HPP_