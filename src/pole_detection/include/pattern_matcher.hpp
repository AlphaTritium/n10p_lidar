#ifndef POLE_DETECTION__PATTERN_MATCHER_HPP_
#define POLE_DETECTION__PATTERN_MATCHER_HPP_

#include "types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <map>

namespace pole_detection
{

struct PatternMatchResult
{
  int matches;
  int total_pairs;
  double match_ratio;
  std::vector<std::pair<int, int>> matched_pairs;  // Track which pairs matched
  std::map<std::pair<int, int>, int> matched_harmonics;  // Which harmonic matched (1×, 2×, etc.)
};

class PatternMatcher
{
public:
  struct Config {
    std::vector<double> expected_distances;  // Base distances (e.g., [0.185])
    double distance_tolerance;
    bool enable_harmonics;  // Match multiples: 1×, 2×, 3× base distance
    int max_harmonic;       // Maximum harmonic to check (default: 3)
    bool publish_debug;
    
    Config()
      : expected_distances({0.185})
      , distance_tolerance(0.03)
      , enable_harmonics(true)
      , max_harmonic(3)
      , publish_debug(false)
    {}
  };
  
  explicit PatternMatcher(rclcpp::Node::SharedPtr node, const Config& config = Config());
  
  PatternMatchResult match(const std::vector<TrackedPole>& poles);
  
  void setConfig(const Config& config) { config_ = config; }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr matrix_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr matches_pub_;
  
  void publishDistanceMatrix(
    const std::map<std::pair<int, int>, double>& distances,
    size_t num_poles);
  
  void publishMatchMarkers(
    const std::map<std::pair<int, int>, double>& distances,
    const std::vector<TrackedPole>& poles);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__PATTERN_MATCHER_HPP_