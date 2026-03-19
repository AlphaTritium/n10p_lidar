#ifndef POLE_DETECTION__GEOMETRIC_PATTERN_MATCHER_HPP_
#define POLE_DETECTION__GEOMETRIC_PATTERN_MATCHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <Eigen/Dense>

namespace pcl {
  template<typename T> class PointCloud;
  struct PointXYZI;
}

namespace pole_detection
{

struct PolePatternResult
{
  bool success = false;
  std::vector<geometry_msgs::msg::Point> pole_positions;
  int num_candidates = 0;
  int inlier_count = 0;
  std::string debug_message;
};

class GeometricPatternMatcher
{
public:
  struct Config {
    double cluster_tolerance;
    int cluster_min_size;
    double ransac_distance_threshold;
    int ransac_max_iterations;
    double ransac_probability;
    int num_poles;
    double expected_spacing;
    double spacing_tolerance;
    double position_tolerance;
    bool use_tracking;
    double smoothing_alpha;
    
    Config()
      : cluster_tolerance(0.03)
      , cluster_min_size(1)
      , ransac_distance_threshold(0.02)
      , ransac_max_iterations(1000)
      , ransac_probability(0.99)
      , num_poles(6)
      , expected_spacing(0.185)
      , spacing_tolerance(0.02)
      , position_tolerance(0.03)
      , use_tracking(true)
      , smoothing_alpha(0.3)
    {}
  };
  
  explicit GeometricPatternMatcher(rclcpp::Node::SharedPtr node, const Config& config);
  
  PolePatternResult detectPolePattern(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);
  
  void setConfig(const Config& config) { config_ = config; }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  std::vector<geometry_msgs::msg::Point> tracked_positions;
  std::vector<int> track_ages;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  
  std::vector<geometry_msgs::msg::Point> extractClusterCentroids(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
  
  bool fitLineLeastSquares(
    const std::vector<geometry_msgs::msg::Point>& candidates,
    Eigen::Vector3f& line_point,
    Eigen::Vector3f& line_direction,
    std::vector<int>& inlier_indices);
  
  std::vector<double> projectPointsToLine(
    const std::vector<geometry_msgs::msg::Point>& points,
    const Eigen::Vector3f& line_point,
    const Eigen::Vector3f& line_direction);
  
  bool findEquallySpacedPattern(
    const std::vector<double>& positions_1d,
    double& start_offset,
    std::vector<int>& matched_indices);
  
  std::vector<geometry_msgs::msg::Point> reconstructPolePositions(
    const Eigen::Vector3f& line_point,
    const Eigen::Vector3f& line_direction,
    double start_offset);
  
  std::vector<geometry_msgs::msg::Point> applyTemporalSmoothing(
    const std::vector<geometry_msgs::msg::Point>& new_positions);
  
  void publishDebugVisualization(
    const PolePatternResult& result,
    const std_msgs::msg::Header& header);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__GEOMETRIC_PATTERN_MATCHER_HPP_