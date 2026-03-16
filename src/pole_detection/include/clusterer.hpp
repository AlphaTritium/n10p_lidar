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
      : cluster_tolerance(0.05)
      , cluster_min_size(6)
      , cluster_max_size(100)
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
  
  void publishDebugMarkers(
    const std::vector<PoleCandidate>& candidates,
    const std_msgs::msg::Header& header);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__CLUSTERER_HPP_