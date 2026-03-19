// include/types.hpp
#ifndef POLE_DETECTION__TYPES_HPP_
#define POLE_DETECTION__TYPES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
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
  
  // Arc-based features for 2D LiDAR
  double arc_length;
  double angular_span;
  double radial_width;
  double curvature_estimate;
  
  // Intensity (optional)
  double avg_intensity;
  double intensity_variance;
  
  // Range from sensor
  double range_from_sensor;
  
  // Legacy radius estimate
  double legacy_radius;
  
  ClusterFeatures()
    : id(0)
    , centroid(geometry_msgs::msg::Point())
    , point_count(0)
    , arc_length(0.0)
    , angular_span(0.0)
    , radial_width(0.0)
    , curvature_estimate(0.0)
    , avg_intensity(0.0)
    , intensity_variance(0.0)
    , range_from_sensor(0.0)
    , legacy_radius(0.0)
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
  
  void update(const geometry_msgs::msg::Point& pos, double confidence, rclcpp::Time stamp, int confirmation_threshold = 5) {
    position.x = 0.8 * position.x + 0.2 * pos.x;
    position.y = 0.8 * position.y + 0.2 * pos.y;
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

}  // namespace pole_detection

#endif  // POLE_DETECTION__TYPES_HPP_