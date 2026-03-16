// include/types.hpp
#ifndef POLE_DETECTION__TYPES_HPP_
#define POLE_DETECTION__TYPES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <string>

namespace pole_detection
{

struct PoleCandidate
{
  int id;
  geometry_msgs::msg::Point centroid;
  double radius;
  int point_count;
  double avg_intensity;
  rclcpp::Time timestamp;
  std::string rejection_reason;  // Empty if valid
  
  bool is_valid() const { return rejection_reason.empty(); }
};

struct TrackedPole
{
  int track_id;
  geometry_msgs::msg::Point position;
  double avg_radius;
  int detection_count;
  int invisible_count;
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  bool is_confirmed;
  bool is_active;
  
  TrackedPole(int id, const geometry_msgs::msg::Point& pos, double radius, rclcpp::Time stamp)
    : track_id(id), position(pos), avg_radius(radius), detection_count(1),
      invisible_count(0), first_seen(stamp), last_seen(stamp),
      is_confirmed(false), is_active(true) {}
  
  void update(const geometry_msgs::msg::Point& pos, double radius, rclcpp::Time stamp) {
    // Exponential moving average
    position.x = 0.8 * position.x + 0.2 * pos.x;
    position.y = 0.8 * position.y + 0.2 * pos.y;
    position.z = pos.z;
    avg_radius = 0.9 * avg_radius + 0.1 * radius;
    last_seen = stamp;
    detection_count++;
    invisible_count = 0;
    if (detection_count >= 5) is_confirmed = true;
  }
  
  void markInvisible() { invisible_count++; }
  bool isStale(int max_invisible = 20) const { return invisible_count > max_invisible; }
};

struct ClusterDebugInfo
{
  int cluster_id;
  int point_count;
  double radius;
  geometry_msgs::msg::Point centroid;
  bool was_accepted;
  std::string reason;
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__TYPES_HPP_