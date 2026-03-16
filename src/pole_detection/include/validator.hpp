#ifndef POLE_DETECTION__validator_HPP_
#define POLE_DETECTION__validator_HPP_

#include "types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace pole_detection
{

class validator
{
public:
  struct Config {
    double expected_radius;
    double radius_tolerance;
    double min_intensity;
    bool use_intensity_stats;
    bool publish_debug;
    
    Config()
      : expected_radius(0.028)
      , radius_tolerance(0.008)
      , min_intensity(50.0)
      , use_intensity_stats(true)
      , publish_debug(false)
    {}
  };
  
  explicit validator(rclcpp::Node::SharedPtr node, const Config& config = Config());
  
  std::vector<PoleCandidate> validate(
    const std::vector<PoleCandidate>& candidates,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
  
  void setConfig(const Config& config) { config_ = config; }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr accepted_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_pub_;
  
  bool validateRadius(const PoleCandidate& candidate);
  bool validateIntensity(const PoleCandidate& candidate, const pcl::PointXYZI& cluster);
  bool validateShape(const PoleCandidate& candidate, const pcl::PointXYZI& cluster);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__validator_HPP_