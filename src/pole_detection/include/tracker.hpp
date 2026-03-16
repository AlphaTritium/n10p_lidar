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
    bool publish_debug_tracks;
    
    Config()
      : max_tracks(6)
      , association_distance(0.15)
      , max_invisible_frames(20)
      , publish_debug_tracks(false)
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