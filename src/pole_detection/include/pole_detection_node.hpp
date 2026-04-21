// ============================================================================
// POLE DETECTION SYSTEM - MAIN NODE HEADER
// ============================================================================
// 
// This header defines the main pole detection node class that implements:
// - ROS2 node with action server interface
// - Point cloud-based pole detection using clustering
// - Multi-object tracking system with EMA smoothing
// - Debug visualization publishers
// - Pattern matching capabilities
//
// KISS Principle: Keep It Simple, Stupid
// Simplified structure with unified implementation
//
// ============================================================================

#ifndef POLE_DETECTION__POLE_DETECTION_NODE_HPP_
#define POLE_DETECTION__POLE_DETECTION_NODE_HPP_

// INCLUDES AND DEPENDENCIES
// ROS2, sensor messages, visualization, and custom headers

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pole_detection/msg/detected_objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include "types.hpp"
#include <pole_detection/action/track_poles.hpp>

namespace pole_detection
{

// ============================================================================
// SECTION 1: CLUSTERER CLASS
// Extracts pole candidates from point clouds using Euclidean clustering
// ============================================================================

class Clusterer
{
public:
  struct Config {
    double cluster_tolerance;      // Distance threshold for clustering (meters)
    int cluster_min_size;         // Minimum points per cluster
    int cluster_max_size;         // Maximum points per cluster
    bool publish_debug_markers;    // Enable debug visualization
    
    Config()
      : cluster_tolerance(0.05)
      , cluster_min_size(6)
      , cluster_max_size(100)
      , publish_debug_markers(false)
    {}
  };
  
  explicit Clusterer(rclcpp::Node::SharedPtr node, const Config& config = Config());
  
  // Extract pole candidates from point cloud
  std::vector<PoleCandidate> extractClusters(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    const std_msgs::msg::Header& header);
  
  void setConfig(const Config& config) { config_ = config; }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  
  // Arc feature extraction methods
  ClusterFeatures extractArcFeatures(
    const pcl::PointCloud<pcl::PointXYZI>& cluster_points,
    const geometry_msgs::msg::Point& centroid);
  
  double computeArcLength(const pcl::PointCloud<pcl::PointXYZI>& sorted_points);
  double computeAngularSpan(const pcl::PointCloud<pcl::PointXYZI>& points, 
                         const geometry_msgs::msg::Point& centroid);
  double computeRadialWidth(const pcl::PointCloud<pcl::PointXYZI>& points);
  double fitCircleCurvature(const pcl::PointCloud<pcl::PointXYZI>& points);
  double computeConvexHullArea(const pcl::PointCloud<pcl::PointXYZI>& points);
  
  void sortPointsByAngle(pcl::PointCloud<pcl::PointXYZI>& points,
                       const geometry_msgs::msg::Point& centroid);
  
  void publishDebugMarkers(const std::vector<PoleCandidate>& candidates,
                       const std_msgs::msg::Header& header);
};

// ============================================================================
// SECTION 2: VALIDATOR CLASS
// Multi-feature validation to filter false positives
// ============================================================================

class validator
{
public:
  struct Config {
    // Multi-feature scoring weights
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
      , publish_debug(false)
    {}
  };
  
  explicit validator(rclcpp::Node::SharedPtr node, const Config& config = Config());
  
  // Validate candidates using multi-feature scoring
  std::vector<PoleCandidate> validate(
    const std::vector<PoleCandidate>& candidates,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
  
  void setConfig(const Config& config) { config_ = config; }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr accepted_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_pub_;
  
  // Multi-feature scoring
  double computeLikelihoodScore(const ClusterFeatures& features);
  std::string generateRejectionReason(double score, const ClusterFeatures& features);
  
  void publishDebugMarkers(const std::vector<PoleCandidate>& accepted,
                       const std::vector<PoleCandidate>& rejected,
                       const rclcpp::Time& timestamp);
};

// ============================================================================
// SECTION 3: PATTERN MATCHER CLASS
// Detects pole line patterns with strict colinearity
// ============================================================================

class PatternMatcher
{
public:
  struct Config {
    std::vector<double> expected_distances;  // Expected pole spacing (meters)
    double distance_tolerance;              // Tolerance for distance matching
    bool enable_harmonics;               // Enable harmonic detection
    int max_harmonic;                   // Maximum harmonic multiple
    bool require_colinear;                // Enforce colinearity
    double colinearity_tolerance;          // Max deviation from line (meters)
    int min_poles_for_pattern;           // Minimum poles for pattern
    bool publish_debug;                  // Enable debug visualization
    
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
  
  // Match pole patterns using distance and colinearity
  PatternMatchResult match(const std::vector<TrackedPole>& poles);
  
  void setConfig(const Config& config) { config_ = config; }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr matrix_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr matches_pub_;
  
  // Strict colinearity methods
  bool arePolesColinear(const std::vector<TrackedPole>& poles) const;
  std::vector<TrackedPole> sortPolesAlongLine(
    const std::vector<TrackedPole>& poles) const;
  std::map<std::pair<int, int>, double> createDistanceMap(
    const std::vector<TrackedPole>& poles) const;
  
  void publishDistanceMatrix(const std::map<std::pair<int, int>, double>& distances,
                         size_t num_poles);
  void publishMatchMarkers(const std::map<std::pair<int, int>, double>& distances,
                        const std::vector<TrackedPole>& poles);
};

// ============================================================================
// SECTION 4: TRACKER CLASS
// Multi-object tracking with EMA smoothing
// ============================================================================

class Tracker
{
public:
  struct Config {
    int max_tracks;                    // Maximum number of tracks
    double association_distance;          // Max distance for track association (meters)
    int max_invisible_frames;           // Max frames before track removal
    int confirmation_threshold;          // Frames to confirm track
    double ema_alpha;                  // EMA smoothing factor (0-1)
    double max_jump_distance;           // Max distance jump before reset (meters)
    bool publish_debug_tracks;          // Enable debug visualization
    
    Config()
      : max_tracks(10)
      , association_distance(0.2)
      , max_invisible_frames(30)
      , confirmation_threshold(3)
      , ema_alpha(0.3)
      , max_jump_distance(0.5)
      , publish_debug_tracks(false)
    {}
  };
  
  explicit Tracker(rclcpp::Node::SharedPtr node, const Config& config = Config());
  
  // Update tracks with new detections
  std::vector<TrackedPole> update(const std::vector<PoleCandidate>& detections,
                                   const std_msgs::msg::Header& header);
  
  // Get current tracks
  std::vector<TrackedPole> getTracks() const { return tracks_; }
  
  void setConfig(const Config& config) { config_ = config; }

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  std::vector<TrackedPole> tracks_;
  int next_track_id_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  
  void publishDebugMarkers(const std::vector<TrackedPole>& tracks,
                       const std_msgs::msg::Header& header);
};

// ============================================================================
// SECTION 5: POLE DETECTION NODE CLASS
// Core class that orchestrates the complete pole detection pipeline
// ============================================================================

class PoleDetectionNode : public rclcpp::Node
{
public:
  PoleDetectionNode();
  
private:
  // Action Server type definitions
  using TrackPoles = pole_detection::action::TrackPoles;
  using GoalHandleTrackPoles = rclcpp_action::ServerGoalHandle<TrackPoles>;
  
  // Action Server components
  rclcpp_action::Server<TrackPoles>::SharedPtr action_server_;
  std::mutex data_mutex_;
  std::atomic<bool> action_active_{false};
  
  // Action callbacks
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const TrackPoles::Goal> goal);
  
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleTrackPoles> goal_handle);
  
  void handle_accepted(const std::shared_ptr<GoalHandleTrackPoles> goal_handle);
  void execute(const std::shared_ptr<GoalHandleTrackPoles> goal_handle);
  void feedbackLoop(const std::shared_ptr<GoalHandleTrackPoles> goal_handle);
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  
  // Publishers
  rclcpp::Publisher<pole_detection::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<pole_detection::msg::DetectedObjects>::SharedPtr poles_pub_;
  
  // Processing modules
  std::unique_ptr<Clusterer> clusterer_;
  std::unique_ptr<validator> validator_;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<PatternMatcher> pattern_matcher_;
  
  // Initialization
  bool modules_initialized_;
  void ensureModulesInitialized();
  void initializeModules();
  
  // Main processing callback
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
};

}  // namespace pole_detection

#endif  // POLE_DETECTION__POLE_DETECTION_NODE_HPP_