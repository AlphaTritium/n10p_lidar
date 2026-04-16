#ifndef POLE_DETECTION__POLE_DETECTION_NODE_HPP_
#define POLE_DETECTION__POLE_DETECTION_NODE_HPP_

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

class PoleDetectionNode : public rclcpp::Node
{
public:
  PoleDetectionNode();
  
private:
  using TrackPoles = pole_detection::action::TrackPoles;
  using GoalHandleTrackPoles = rclcpp_action::ServerGoalHandle<TrackPoles>;
  
  rclcpp_action::Server<TrackPoles>::SharedPtr action_server_;
  std::atomic<bool> action_active_{false};
  
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&, 
      std::shared_ptr<const TrackPoles::Goal>);
  
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleTrackPoles>);
  
  void handle_accepted(const std::shared_ptr<GoalHandleTrackPoles>);
  void execute(const std::shared_ptr<GoalHandleTrackPoles>);
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr poles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr validated_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pattern_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pipeline_debug_pub_;
  
  std::unique_ptr<Tracker> tracker_;
  
  void ensureTrackerInitialized();
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__POLE_DETECTION_NODE_HPP_