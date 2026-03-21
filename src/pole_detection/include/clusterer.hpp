#ifndef POLE_DETECTION__CLUSTERER_HPP_
#define POLE_DETECTION__CLUSTERER_HPP_

#include "types.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace pole_detection
{

class Clusterer
{
public:
  struct Config {
    double cluster_tolerance;
    int cluster_min_size;
    int cluster_max_size;
    bool publish_debug_markers;
    
    Config()
      : cluster_tolerance(0.04)
      , cluster_min_size(3)
      , cluster_max_size(30)
      , publish_debug_markers(false)
    {}
  };
  
  explicit Clusterer(rclcpp::Node::SharedPtr node, const Config& config = Config());
  
  std::vector<PoleCandidate> extractClusters(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    const std_msgs::msg::Header& header);
  
  void setConfig(const Config& config) { config_ = config; }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  
  // NEW: Arc feature extraction methods
  ClusterFeatures extractArcFeatures(
    const pcl::PointCloud<pcl::PointXYZI>& cluster_points,
    const geometry_msgs::msg::Point& centroid);
  
  double computeArcLength(const pcl::PointCloud<pcl::PointXYZI>& sorted_points);
  double computeAngularSpan(const pcl::PointCloud<pcl::PointXYZI>& points, const geometry_msgs::msg::Point& centroid);
  double computeRadialWidth(const pcl::PointCloud<pcl::PointXYZI>& points);
  double fitCircleCurvature(const pcl::PointCloud<pcl::PointXYZI>& points);
  
  void sortPointsByAngle(
    pcl::PointCloud<pcl::PointXYZI>& points,
    const geometry_msgs::msg::Point& centroid);
  
  void publishDebugMarkers(
    const std::vector<PoleCandidate>& candidates,
    const std_msgs::msg::Header& header);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__CLUSTERER_HPP_