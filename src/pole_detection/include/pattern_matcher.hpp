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
  std::vector<std::pair<int, int>> matched_pairs;
  std::map<std::pair<int, int>, int> matched_harmonics;
};

class PatternMatcher
{
public:
  struct Config {
    std::vector<double> expected_distances;
    double distance_tolerance;
    bool enable_harmonics;
    int max_harmonic;
    bool require_colinear;
    double colinearity_tolerance;
    int min_poles_for_pattern;
    bool publish_debug;
    
    Config()
      : expected_distances({0.185})
      , distance_tolerance(0.015)
      , enable_harmonics(false)
      , max_harmonic(1)
      , require_colinear(true)
      , colinearity_tolerance(0.02)
      , min_poles_for_pattern(4)
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
  
  // NEW: Strict colinearity methods
  bool arePolesColinear(const std::vector<TrackedPole>& poles) const;
  std::vector<TrackedPole> sortPolesAlongLine(const std::vector<TrackedPole>& poles) const;
  std::map<std::pair<int, int>, double> createDistanceMap(const std::vector<TrackedPole>& poles) const;
  
  void publishDistanceMatrix(
    const std::map<std::pair<int, int>, double>& distances,
    size_t num_poles);
  
  void publishMatchMarkers(
    const std::map<std::pair<int, int>, double>& distances,
    const std::vector<TrackedPole>& poles);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__PATTERN_MATCHER_HPP_