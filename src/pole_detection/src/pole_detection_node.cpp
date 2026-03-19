#include "pole_detection_node.hpp"
#include "pattern_matcher.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace pole_detection
{

PoleDetectionNode::PoleDetectionNode()
  : Node("pole_detection"), modules_initialized_(false)
{
  declare_parameter("cluster_tolerance", 0.05);
  declare_parameter("cluster_min_size", 3);
  declare_parameter("cluster_max_size", 50);
  declare_parameter("publish_debug_clusters", true);
  
  declare_parameter("expected_radius", 0.028);
  declare_parameter("radius_tolerance", 0.008);
  declare_parameter("publish_debug_validation", true);
  
  declare_parameter("max_tracks", 10);
  declare_parameter("association_distance", 0.2);
  declare_parameter("max_invisible_frames", 30);
  declare_parameter("publish_debug_tracks", true);
  
  declare_parameter("enable_pattern_matching", true);
  declare_parameter("publish_debug_pattern", true);
  
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lslidar_point_cloud", 10,
    std::bind(&PoleDetectionNode::cloudCallback, this, std::placeholders::_1));
  
  objects_pub_ = create_publisher<lslidar_msgs::msg::DetectedObjects>(
    "/detected_objects", 10);
  poles_pub_ = create_publisher<lslidar_msgs::msg::DetectedObjects>(
    "/detected_poles", 10);
  
  RCLCPP_INFO(get_logger(), "Pole Detection Node initialized");
  RCLCPP_INFO(get_logger(), "Pipeline: Clusterer → Validator → Tracker → PatternMatcher");
}

void PoleDetectionNode::ensureModulesInitialized()
{
  if (modules_initialized_) return;
  initializeModules();
  modules_initialized_ = true;
  RCLCPP_INFO(get_logger(), "All modules initialized");
}

void PoleDetectionNode::initializeModules()
{
  auto shared_node = shared_from_this();
  
  // Clusterer - LOW threshold for sparse data
  Clusterer::Config cluster_config;
  cluster_config.cluster_tolerance = get_parameter("cluster_tolerance").as_double();
  cluster_config.cluster_min_size = get_parameter("cluster_min_size").as_int();
  cluster_config.cluster_max_size = get_parameter("cluster_max_size").as_int();
  cluster_config.publish_debug_markers = get_parameter("publish_debug_clusters").as_bool();
  clusterer_ = std::make_unique<Clusterer>(shared_node, cluster_config);
  
  // Validator - SIMPLE range/size check
  validator::Config validator_config;
  validator_config.min_angular_span = 15.0;      // Use existing parameter
  validator_config.max_angular_span = 70.0;
  validator_config.min_point_count = 3;           // Reduced for sparse data
  validator_config.max_point_count = 50;
  validator_config.min_radial_width = 0.005;     // 5mm minimum
  validator_config.max_radial_width = 0.1;       // 10cm maximum (was max_radial_extent)
  validator_config.max_range = 1.0s;              // Maximum range
  // Note: no min_range in validator, so we'll rely on clustering to filter close points
  validator_config.publish_debug = get_parameter("publish_debug_validation").as_bool();
  validator_ = std::make_unique<validator>(shared_node, validator_config);
  
  // Tracker - AGGRESSIVE confirmation
  Tracker::Config tracker_config;
  tracker_config.max_tracks = get_parameter("max_tracks").as_int();
  tracker_config.association_distance = get_parameter("association_distance").as_double();
  tracker_config.max_invisible_frames = get_parameter("max_invisible_frames").as_int();
  tracker_config.confirmation_threshold = 3;
  tracker_config.publish_debug_tracks = get_parameter("publish_debug_tracks").as_bool();
  tracker_ = std::make_unique<Tracker>(shared_node, tracker_config);
  
  // PatternMatcher - WITH HARMONICS
  PatternMatcher::Config pattern_config;
  pattern_config.expected_distances = {0.185};
  pattern_config.distance_tolerance = 0.05;
  pattern_config.enable_harmonics = true;
  pattern_config.max_harmonic = 3;
  pattern_config.publish_debug = get_parameter("publish_debug_pattern").as_bool();
  pattern_matcher_ = std::make_unique<PatternMatcher>(shared_node, pattern_config);
  
  RCLCPP_INFO(get_logger(), "===========================================");
  RCLCPP_INFO(get_logger(), "Config: cluster_min=%d, track_confirm=%d", 
              cluster_config.cluster_min_size, tracker_config.confirmation_threshold);
  RCLCPP_INFO(get_logger(), "Pattern matching: harmonics ENABLED (±5cm)");
  RCLCPP_INFO(get_logger(), "===========================================");
}

void PoleDetectionNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  ensureModulesInitialized();
  
  // Convert raw cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  try {
    pcl::fromROSMsg(*msg, *cloud);
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "Cloud conversion failed: %s", e.what());
    return;
  }
  
  if (cloud->empty()) {
    RCLCPP_DEBUG(get_logger(), "Empty cloud");
    return;
  }
  
  // Stage 1: Clustering
  auto candidates = clusterer_->extractClusters(cloud, msg->header);
  
  // Stage 2: Validation
  auto validated = validator_->validate(candidates, cloud);
  
  // Stage 3: Tracking
  auto tracked = tracker_->update(validated, msg->header);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "Tracking %zu poles", tracked.size());
  
  // Stage 4: Pattern Matching (WITH HARMONICS)
  if (!tracked.empty() && tracked.size() >= 2) {
    auto match_result = pattern_matcher_->match(tracked);
    
    if (match_result.match_ratio > 0.5) {
      RCLCPP_INFO(get_logger(),
        "✓ Pattern match: %.1f%% (%d/%d pairs)",
        match_result.match_ratio * 100.0, match_result.matches, match_result.total_pairs);
    }
  }
  
  // Publish
  if (!tracked.empty()) {
    lslidar_msgs::msg::DetectedObjects poles_msg;
    poles_msg.header = msg->header;
    
    for (const auto& pole : tracked) {
      lslidar_msgs::msg::DetectedObject obj;
      obj.label = "pole_" + std::to_string(pole.track_id);
      obj.x = pole.position.x;
      obj.y = pole.position.y;
      obj.z = 0.0;
      obj.confidence = std::min(1.0, pole.detection_count / 10.0);
      poles_msg.objects.push_back(obj);
    }
    
    poles_pub_->publish(poles_msg);
    objects_pub_->publish(poles_msg);
  }
}

}  // namespace pole_detection

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pole_detection::PoleDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}