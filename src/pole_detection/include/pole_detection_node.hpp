#ifndef POLE_DETECTION__POLE_DETECTION_NODE_HPP_
#define POLE_DETECTION__POLE_DETECTION_NODE_HPP_

#include "clusterer.hpp"
#include "validator.hpp"
#include "tracker.hpp"
#include "pattern_matcher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lslidar_msgs/msg/detected_objects.hpp>

namespace pole_detection
{

class PoleDetectionNode : public rclcpp::Node
{
public:
  PoleDetectionNode();
  
private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  
  // Publishers
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr poles_pub_;
  
  // Pipeline modules (Preprocessor removed - using raw cloud directly)
  std::unique_ptr<Clusterer> clusterer_;
  std::unique_ptr<validator> validator_;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<PatternMatcher> pattern_matcher_;
  
  bool modules_initialized_ = false;
  
  // Methods
  void ensureModulesInitialized();
  void initializeModules();
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__POLE_DETECTION_NODE_HPP_