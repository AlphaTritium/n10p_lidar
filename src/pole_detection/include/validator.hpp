#ifndef POLE_DETECTION__VALIDATOR_HPP_
#define POLE_DETECTION__VALIDATOR_HPP_

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
    // NEW: Multi-feature scoring weights
    double weight_angular_span;
    double weight_point_count;
    double weight_radial_width;
    double weight_curvature;
    double weight_range;
    
    // Thresholds
    double acceptance_threshold;      // Score ≥ this → accept
    double min_angular_span;
    double max_angular_span;
    int min_point_count;
    int max_point_count;
    double min_radial_width;
    double max_radial_width;
    double max_range;
    
    // Legacy (deprecated but kept for compatibility)
    double expected_radius;
    double radius_tolerance;
    bool use_intensity_stats;
    double min_intensity;
    
    bool publish_debug;
    
    Config()
      : weight_angular_span(0.30)
      , weight_point_count(0.20)
      , weight_radial_width(0.25)
      , weight_curvature(0.15)
      , weight_range(0.10)
      , acceptance_threshold(0.60)
      , min_angular_span(15.0)
      , max_angular_span(70.0)
      , min_point_count(6)
      , max_point_count(50)
      , min_radial_width(0.005)
      , max_radial_width(0.025)
      , max_range(0.8)
      , expected_radius(0.028)
      , radius_tolerance(0.008)
      , use_intensity_stats(false)
      , min_intensity(50.0)
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
  
  // NEW: Multi-feature scoring
  double computeLikelihoodScore(const ClusterFeatures& features);
  std::string generateRejectionReason(double score, const ClusterFeatures& features);
  
  // Legacy (deprecated)
  bool validateRadius(const PoleCandidate& candidate);
  bool validateIntensity(const PoleCandidate& candidate, const pcl::PointXYZI& cluster);
  bool validateShape(const PoleCandidate& candidate, const pcl::PointXYZI& cluster);
  
  void publishDebugMarkers(
    const std::vector<PoleCandidate>& accepted,
    const std::vector<PoleCandidate>& rejected,
    const rclcpp::Time& timestamp);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__VALIDATOR_HPP_