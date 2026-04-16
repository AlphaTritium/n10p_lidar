#ifndef POLE_DETECTION__POLE_DETECTION_NODE_HPP_
#define POLE_DETECTION__POLE_DETECTION_NODE_HPP_

#include "clusterer.hpp"
#include "validator.hpp"
#include "tracker.hpp"
#include "pattern_matcher.hpp"
#include <rclcpp/rclcpp.hpp>
#include "geometric_pattern_matcher.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <sstream>
#include <pole_detection/action/track_poles.hpp>

namespace pole_detection
{

class PoleDetectionNode : public rclcpp::Node
{
public:
  PoleDetectionNode();
  
private:
  // Action Server (from rc2026_head_finder)
  using TrackPoles = pole_detection::action::TrackPoles;
  using GoalHandleTrackPoles = rclcpp_action::ServerGoalHandle<TrackPoles>;
  
  rclcpp_action::Server<TrackPoles>::SharedPtr action_server_;
  std::mutex data_mutex_;
  std::atomic<bool> action_active_{false};
  
  // Action callbacks
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid, 
      std::shared_ptr<const TrackPoles::Goal> goal);
  
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleTrackPoles> goal_handle);
  
  void handle_accepted(const std::shared_ptr<GoalHandleTrackPoles> goal_handle);
  void execute(const std::shared_ptr<GoalHandleTrackPoles> goal_handle);
  void feedbackLoop(const std::shared_ptr<GoalHandleTrackPoles> goal_handle);
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  
  // Publishers
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr poles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pipeline_debug_pub_;
  
  // Pipeline modules (Preprocessor removed - using raw cloud directly)
  std::unique_ptr<Clusterer> clusterer_;
  std::unique_ptr<validator> validator_;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<PatternMatcher> pattern_matcher_;
  std::unique_ptr<GeometricPatternMatcher> geometric_matcher_;
  
  bool modules_initialized_ = false;
  
  // Methods
  void ensureModulesInitialized();
  void initializeModules();
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void publishPipelineDebug(
    const std::vector<PoleCandidate>& candidates,
    const std::vector<PoleCandidate>& validated,
    const std::vector<TrackedPole>& tracked,
    const PatternMatchResult& match_result,
    const std_msgs::msg::Header& header);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__POLE_DETECTION_NODE_HPP_