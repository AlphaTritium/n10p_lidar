#ifndef POLE_DETECTION__TRACKER_HPP_
#define POLE_DETECTION__TRACKER_HPP_

#include "types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>

namespace pole_detection
{

class Tracker
{
public:
  struct Config {
    int max_tracks;
    double association_distance;
    int max_invisible_frames;
    int confirmation_threshold;
    bool publish_debug_tracks;
    
    // New parameters from rc2026_head_finder
    double ema_alpha;           // EMA smoothing factor (0.0-1.0)
    double max_jump_distance;    // Jump detection threshold (meters)
    
    Config()
      : max_tracks(6)
      , association_distance(0.15)
      , max_invisible_frames(20)
      , confirmation_threshold(3)
      , publish_debug_tracks(false)
      , ema_alpha(0.3)           // 30% new, 70% old (from rc2026_head_finder)
      , max_jump_distance(0.5)     // 50cm jump threshold
    {}
  };
  
  explicit Tracker(rclcpp::Node::SharedPtr node, const Config& config = Config());
  
  std::vector<TrackedPole> update(
    const std::vector<PoleCandidate>& detections,
    const std_msgs::msg::Header& header);
  
  const std::vector<TrackedPole>& getTracks() const { return tracks_; }
  void clear() { tracks_.clear(); }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  std::vector<TrackedPole> tracks_;
  int next_track_id_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  
  int associateTrack(const PoleCandidate& detection);
  void publishDebugMarkers(
    const std::vector<TrackedPole>& tracks,
    const std_msgs::msg::Header& header);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__TRACKER_HPP_