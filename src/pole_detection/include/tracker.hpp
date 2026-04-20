// ============================================================================
// POLE DETECTION SYSTEM - MULTI-OBJECT TRACKER HEADER
// ============================================================================
// 
// This header defines the multi-object tracking system that implements:
// - Detection-to-track association using distance-based matching
// - Exponential Moving Average (EMA) smoothing for stable tracking
// - Jump detection to handle sudden position changes
// - Track lifecycle management (creation, confirmation, removal)
// - Debug visualization for track monitoring
//
// ============================================================================

#ifndef POLE_DETECTION__TRACKER_HPP_
#define POLE_DETECTION__TRACKER_HPP_

// INCLUDES AND DEPENDENCIES
// Custom types and ROS2 visualization components

#include "types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>

namespace pole_detection
{

// ============================================================================
// SECTION 2: TRACKER CONFIGURATION STRUCTURE
// ============================================================================
// Configuration parameters for track behavior and performance
// ============================================================================

struct Config {
  // ==========================================================================
  // SECTION 2.1: TRACK MANAGEMENT PARAMETERS
  // ==========================================================================
  
  int max_tracks;                    // Maximum number of concurrent tracks
  double association_distance;       // Maximum distance for detection association
  int max_invisible_frames;          // Frames before removing invisible tracks
  int confirmation_threshold;        // Frames needed for track confirmation
  bool publish_debug_tracks;         // Enable debug visualization
  
  // ==========================================================================
  // SECTION 2.2: ADVANCED TRACKING PARAMETERS (from rc2026_head_finder)
  // ==========================================================================
  
  double ema_alpha;                  // EMA smoothing factor (0.0-1.0)
  double max_jump_distance;          // Jump detection threshold (meters)
  
  // ==========================================================================
  // SECTION 2.3: DEFAULT CONFIGURATION VALUES
  // ==========================================================================
  
  Config()
    : max_tracks(6)
    , association_distance(0.15)
    , max_invisible_frames(20)
    , confirmation_threshold(3)
    , publish_debug_tracks(false)
    , ema_alpha(0.3)                 // 30% new, 70% old (from rc2026_head_finder)
    , max_jump_distance(0.5)         // 50cm jump threshold
  {}
};

// ============================================================================
// SECTION 3: MAIN TRACKER CLASS DEFINITION
// ============================================================================
// Core class implementing multi-object tracking with EMA smoothing
// ============================================================================

class Tracker
{
public:
  // ==========================================================================
  // SECTION 3.1: CONSTRUCTOR AND INITIALIZATION
  // ==========================================================================
  
  explicit Tracker(rclcpp::Node::SharedPtr node, const Config& config = Config());
  
  // ==========================================================================
  // SECTION 3.2: MAIN TRACK UPDATE INTERFACE
  // ==========================================================================
  
  std::vector<TrackedPole> update(
    const std::vector<PoleCandidate>& detections,
    const std_msgs::msg::Header& header);
  
  // ==========================================================================
  // SECTION 3.3: TRACK ACCESS AND MANAGEMENT
  // ==========================================================================
  
  const std::vector<TrackedPole>& getTracks() const { return tracks_; }
  void clear() { tracks_.clear(); }

private:
  // ==========================================================================
  // SECTION 3.4: INTERNAL COMPONENTS AND STATE
  // ==========================================================================
  
  rclcpp::Node::SharedPtr node_;
  Config config_;
  std::vector<TrackedPole> tracks_;
  int next_track_id_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  
  // ==========================================================================
  // SECTION 3.5: INTERNAL PROCESSING METHODS
  // ==========================================================================
  
  int associateTrack(const PoleCandidate& detection);
  void publishDebugMarkers(
    const std::vector<TrackedPole>& tracks,
    const std_msgs::msg::Header& header);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__TRACKER_HPP_