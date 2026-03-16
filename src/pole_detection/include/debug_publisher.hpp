#ifndef POLE_DETECTION__DEBUG_PUBLISHER_HPP_
#define POLE_DETECTION__DEBUG_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>

namespace pole_detection
{

/**
 * @brief Utility class for publishing debug visualizations
 * 
 * Provides common marker creation functions used across all modules.
 * Each module can use this to publish its debug output consistently.
 */
class DebugPublisher
{
public:
  explicit DebugPublisher(rclcpp::Node::SharedPtr node, bool enable_debug = false);
  
  // Point cloud debug
  void publishCloud(
    const std::string& topic,
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    const std_msgs::msg::Header& header);
  
  // Marker utilities
  visualization_msgs::msg::Marker createSphereMarker(
    const geometry_msgs::msg::Point& position,
    double scale,
    const std::string& ns,
    int id,
    float r, float g, float b, float a,
    const std_msgs::msg::Header& header,
    rclcpp::Duration lifetime = rclcpp::Duration::from_seconds(0.5));
  
  visualization_msgs::msg::Marker createTextMarker(
    const geometry_msgs::msg::Point& position,
    const std::string& text,
    const std::string& ns,
    int id,
    double scale,
    const std_msgs::msg::Header& header,
    rclcpp::Duration lifetime = rclcpp::Duration::from_seconds(0.5));
  
  visualization_msgs::msg::Marker createLineMarker(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2,
    const std::string& ns,
    int id,
    float r, float g, float b,
    const std_msgs::msg::Header& header,
    rclcpp::Duration lifetime = rclcpp::Duration::from_seconds(0.5));
  
  // Publish marker arrays
  void publishMarkers(const visualization_msgs::msg::MarkerArray& markers);
  
  // Enable/disable debug publishing
  void setEnabled(bool enable) { enabled_ = enable; }
  bool isEnabled() const { return enabled_; }

private:
  rclcpp::Node::SharedPtr node_;
  bool enabled_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__DEBUG_PUBLISHER_HPP_