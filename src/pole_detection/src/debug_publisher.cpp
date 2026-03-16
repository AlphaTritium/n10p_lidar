#include "debug_publisher.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace pole_detection
{

DebugPublisher::DebugPublisher(rclcpp::Node::SharedPtr node, bool enable_debug)
  : node_(node), enabled_(enable_debug)
{
  if (enabled_) {
    cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/debug/cloud", 10);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/markers", 10);
    RCLCPP_INFO(node_->get_logger(), "DebugPublisher initialized");
  }
}

void DebugPublisher::publishCloud(
  const std::string& topic,
  const pcl::PointCloud<pcl::PointXYZI>& cloud,
  const std_msgs::msg::Header& header)
{
  if (!enabled_ || !cloud_pub_) return;
  
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = header;
  cloud_pub_->publish(msg);
}

visualization_msgs::msg::Marker DebugPublisher::createSphereMarker(
  const geometry_msgs::msg::Point& position,
  double scale,
  const std::string& ns,
  int id,
  float r, float g, float b, float a,
  const std_msgs::msg::Header& header,
  rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = position;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.lifetime = lifetime;
  return marker;
}

visualization_msgs::msg::Marker DebugPublisher::createTextMarker(
  const geometry_msgs::msg::Point& position,
  const std::string& text,
  const std::string& ns,
  int id,
  double scale,
  const std_msgs::msg::Header& header,
  rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = position;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = scale;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.text = text;
  marker.lifetime = lifetime;
  return marker;
}

visualization_msgs::msg::Marker DebugPublisher::createLineMarker(
  const geometry_msgs::msg::Point& p1,
  const geometry_msgs::msg::Point& p2,
  const std::string& ns,
  int id,
  float r, float g, float b,
  const std_msgs::msg::Header& header,
  rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.scale.x = 0.01;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = lifetime;
  return marker;
}

void DebugPublisher::publishMarkers(const visualization_msgs::msg::MarkerArray& markers)
{
  if (!enabled_ || !marker_pub_ || markers.markers.empty()) return;
  marker_pub_->publish(markers);
}

}  // namespace pole_detection