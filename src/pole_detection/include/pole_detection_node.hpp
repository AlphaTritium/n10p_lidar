// ============================================================================
// POLE DETECTION SYSTEM - MAIN NODE HEADER (LASERSCAN VERSION)
// ============================================================================
// 
// This header defines the main pole detection node class that implements:
// - ROS2 node with action server interface
// - LaserScan-based pole detection using nearest neighbor clustering
// - Multi-object tracking system with EMA smoothing
// - Debug visualization publishers
// - Pattern matching capabilities
//
// ============================================================================

#ifndef POLE_DETECTION__POLE_DETECTION_NODE_HPP_
#define POLE_DETECTION__POLE_DETECTION_NODE_HPP_

// INCLUDES AND DEPENDENCIES
// ROS2, sensor messages, visualization, and custom headers

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pole_detection/msg/detected_objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include <optional>
#include <pole_detection/action/track_poles.hpp>

namespace pole_detection
{

// ============================================================================
// SECTION 1: POINT2D STRUCTURE
// ============================================================================
// Simple 2D point structure for LaserScan processing
// ============================================================================

struct Point2D {
  double x;
  double y;
};

// ============================================================================
// SECTION 2: TRACKED POLE STRUCTURE
// ============================================================================
// Structure to maintain tracking state for each detected pole
// ============================================================================

struct TrackedPole {
  int track_id;
  Point2D position;
  int detection_count;
  int invisible_count;
  bool is_visible;
  bool is_confirmed;
  builtin_interfaces::msg::Time last_seen;
};

// ============================================================================
// SECTION 3: MAIN POLE DETECTION NODE CLASS DEFINITION
// ============================================================================
// Core class that orchestrates the complete pole detection pipeline
// ============================================================================

class PoleDetectionNode : public rclcpp::Node
{
public:
  // CONSTRUCTOR AND INITIALIZATION  
  PoleDetectionNode();
  
private:
  // ACTION SERVER TYPE DEFINITIONS
  
  using TrackPoles = pole_detection::action::TrackPoles;
  using GoalHandleTrackPoles = rclcpp_action::ServerGoalHandle<TrackPoles>;
  
  // ACTION SERVER COMPONENTS

  rclcpp_action::Server<TrackPoles>::SharedPtr action_server_;
  std::atomic<bool> action_active_{false};
  
  // Action server callback handlers
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&, 
      std::shared_ptr<const TrackPoles::Goal>);
  
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleTrackPoles>);
  
  void handle_accepted(const std::shared_ptr<GoalHandleTrackPoles>);
  void execute(const std::shared_ptr<GoalHandleTrackPoles>);
  
  // SENSOR INPUT AND DATA PROCESSING
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  // OUTPUT PUBLISHERS FOR DETECTIONS AND VISUALIZATION
  
  // Main detection outputs
  rclcpp::Publisher<pole_detection::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<pole_detection::msg::DetectedObjects>::SharedPtr poles_pub_;
  
  // Unified debug visualization (rc2026_head_finder inspired)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_visuals_pub_;
  
  // TRACKING SYSTEM COMPONENTS
  
  std::vector<TrackedPole> tracks_;
  int next_track_id_;
  std::mutex data_mutex_;
  
  // TRACKING PARAMETERS
  
  double max_search_distance_;
  double cluster_tolerance_;
  int min_points_per_cluster_;
  double ema_alpha_;
  double max_jump_distance_;
  int max_tracks_;
  double association_distance_;
  
  // PROCESSING METHODS
  
  std::vector<Point2D> ExtractClusterCenters(const sensor_msgs::msg::LaserScan::SharedPtr& msg);
  std::optional<std::pair<Point2D, Point2D>> FitLinePCA(const std::vector<Point2D>& points);
  void UpdateTracks(const std::vector<Point2D>& detections, const builtin_interfaces::msg::Time& stamp);
  void MarkAllTracksInvisible();
  
  // VISUALIZATION METHODS
  
  void PublishVisualizations(const std::vector<Point2D>& centers, 
                       const std::optional<std::pair<Point2D, Point2D>>& line, 
                       const std::optional<std::vector<Point2D>>& track_positions,
                       const std::string& frame_id);
  void PublishDetectionResults(const std_msgs::msg::Header& header);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__POLE_DETECTION_NODE_HPP_