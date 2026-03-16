#ifndef POLE_DETECTION__PREPROCESSOR_HPP_
#define POLE_DETECTION__PREPROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pole_detection
{

class Preprocessor
{
public:
  struct Config {
    double range_min;
    double range_max;
    double z_min;
    double z_max;
    double voxel_leaf_size;
    bool use_intensity_filter;
    double min_intensity;
    bool publish_debug_cloud;
    
    Config()
      : range_min(0.2)
      , range_max(0.8)
      , z_min(-0.3)
      , z_max(0.3)
      , voxel_leaf_size(0.01)
      , use_intensity_filter(true)
      , min_intensity(50.0)
      , publish_debug_cloud(false)
    {}
  };
  
  explicit Preprocessor(rclcpp::Node::SharedPtr node, const Config& config = Config());
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr process(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input);
  
  void setConfig(const Config& config) { config_ = config; }
  const Config& getConfig() const { return config_; }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
  
  void publishDebug(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std_msgs::msg::Header& header);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__PREPROCESSOR_HPP_