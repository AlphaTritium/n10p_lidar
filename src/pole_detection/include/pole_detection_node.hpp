// ============================================================================
// POLE DETECTION SYSTEM - MAIN NODE HEADER
// ============================================================================
// 
// This header defines the main pole detection node class that implements:
// - ROS2 node with action server interface
// - Point cloud processing pipeline
// - Multi-object tracking system
// - Debug visualization publishers
// - Pattern matching capabilities
//
// ============================================================================

#ifndef POLE_DETECTION__POLE_DETECTION_NODE_HPP_
#define POLE_DETECTION__POLE_DETECTION_NODE_HPP_

// INCLUDES AND DEPENDENCIES
// ROS2, sensor messages, visualization, and custom headers

#include "tracker.hpp"
#include "types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include <pole_detection/action/track_poles.hpp>

namespace pole_detection
{

// SECTION 2: MAIN POLE DETECTION NODE CLASS DEFINITION
// Core class that orchestrates the complete pole detection pipeline

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
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  
  // OUTPUT PUBLISHERS FOR DETECTIONS AND VISUALIZATION
  
  // Main detection outputs
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr poles_pub_;
  
  // Debug visualization publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr validated_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pattern_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pipeline_debug_pub_;
  
  // TRACKING SYSTEM COMPONENTS
  
  std::unique_ptr<Tracker> tracker_;
  std::mutex tracker_mutex_;
  void ensureTrackerInitialized();
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__POLE_DETECTION_NODE_HPP_