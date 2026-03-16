#ifndef POLE_DETECTION__POLE_DETECTION_NODE_HPP_
#define POLE_DETECTION__POLE_DETECTION_NODE_HPP_

#include "preprocessor.hpp"
#include "clusterer.hpp"
#include "validator.hpp"
#include "tracker.hpp"
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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr poles_pub_;
  
  std::unique_ptr<Preprocessor> preprocessor_;
  std::unique_ptr<Clusterer> clusterer_;
  std::unique_ptr<validator> validator_;
  std::unique_ptr<Tracker> tracker_;
  
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void loadParameters();
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__POLE_DETECTION_NODE_HPP_