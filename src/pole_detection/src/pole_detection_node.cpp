#include "pole_detection_node.hpp"
#include "pattern_matcher.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace pole_detection
{

PoleDetectionNode::PoleDetectionNode()
  : Node("pole_detection")
{
  loadParameters();
  
  // Create subscribers
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lslidar_point_cloud", 10,
    std::bind(&PoleDetectionNode::cloudCallback, this, std::placeholders::_1));
  
  // Create publishers
  objects_pub_ = create_publisher<lslidar_msgs::msg::DetectedObjects>(
    "/detected_objects", 10);
  poles_pub_ = create_publisher<lslidar_msgs::msg::DetectedObjects>(
    "/detected_poles", 10);
  
  RCLCPP_INFO(get_logger(), "Pole Detection Node initialized");
  RCLCPP_INFO(get_logger(), "Pipeline: Preprocessor → Clusterer → validator → Tracker → PatternMatcher");
}

void PoleDetectionNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  // Stage 1: Preprocessing
  auto preprocessed = preprocessor_->process(msg);
  if (!preprocessed || preprocessed->empty()) {
    RCLCPP_DEBUG(get_logger(), "Preprocessing produced empty cloud");
    return;
  }
  
  // Stage 2: Clustering
  auto candidates = clusterer_->extractClusters(preprocessed, msg->header);
  RCLCPP_DEBUG(get_logger(), "Extracted %zu cluster candidates", candidates.size());
  
  // Stage 3: Validation
  auto validated = validator_->validate(candidates, preprocessed);
  RCLCPP_DEBUG(get_logger(), "Validated %zu poles", validated.size());
  
  // Stage 4: Tracking
  auto tracked = tracker_->update(validated, msg->header);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "Tracking %zu poles", tracked.size());
  
  // Stage 5: Pattern Matching (optional)
  if (!tracked.empty() && tracked.size() >= 2) {
    PatternMatcher::Config pm_config;
    pm_config.expected_distances = {0.185};
    pm_config.distance_tolerance = 0.03;
    pm_config.publish_debug = get_parameter("publish_debug_pattern").as_bool();
    
    PatternMatcher pattern_matcher(shared_from_this(), pm_config);
    auto match_result = pattern_matcher.match(tracked);
    
    if (match_result.match_ratio > 0.5) {
      RCLCPP_INFO(get_logger(),
        "✓ Pattern match successful: %.1f%% of distances match expected configuration",
        match_result.match_ratio * 100.0);
    }
  }
  
  // Publish final output
  if (!tracked.empty()) {
    lslidar_msgs::msg::DetectedObjects poles_msg;
    poles_msg.header = msg->header;
    
    for (const auto& pole : tracked) {
      lslidar_msgs::msg::DetectedObject obj;
      obj.label = "pole_" + std::to_string(pole.track_id);
      obj.x = pole.position.x;
      obj.y = pole.position.y;
      obj.z = pole.position.z;
      obj.confidence = std::min(1.0, pole.detection_count / 20.0);
      poles_msg.objects.push_back(obj);
    }
    
    poles_pub_->publish(poles_msg);
    
    // Also publish to generic objects topic for backward compatibility
    lslidar_msgs::msg::DetectedObjects objects_msg = poles_msg;
    objects_pub_->publish(objects_msg);
  }
}

void PoleDetectionNode::loadParameters()
{
  // Preprocessor parameters
  Preprocessor::Config preproc_config;
  declare_parameter("range_min", 0.2);
  declare_parameter("range_max", 0.8);
  declare_parameter("z_min", -0.3);
  declare_parameter("z_max", 0.3);
  declare_parameter("voxel_leaf_size", 0.01);
  declare_parameter("use_intensity_filter", true);
  declare_parameter("min_intensity", 50.0);
  declare_parameter("publish_debug_cloud", false);
  
  preproc_config.range_min = get_parameter("range_min").as_double();
  preproc_config.range_max = get_parameter("range_max").as_double();
  preproc_config.z_min = get_parameter("z_min").as_double();
  preproc_config.z_max = get_parameter("z_max").as_double();
  preproc_config.voxel_leaf_size = get_parameter("voxel_leaf_size").as_double();
  preproc_config.use_intensity_filter = get_parameter("use_intensity_filter").as_bool();
  preproc_config.min_intensity = get_parameter("min_intensity").as_double();
  preproc_config.publish_debug_cloud = get_parameter("publish_debug_cloud").as_bool();
  
  preprocessor_ = std::make_unique<Preprocessor>(shared_from_this(), preproc_config);
  
  // Clusterer parameters
  Clusterer::Config cluster_config;
  declare_parameter("cluster_tolerance", 0.05);
  declare_parameter("cluster_min_size", 6);
  declare_parameter("cluster_max_size", 100);
  declare_parameter("publish_debug_clusters", false);
  
  cluster_config.cluster_tolerance = get_parameter("cluster_tolerance").as_double();
  cluster_config.cluster_min_size = get_parameter("cluster_min_size").as_int();
  cluster_config.cluster_max_size = get_parameter("cluster_max_size").as_int();
  cluster_config.publish_debug_markers = get_parameter("publish_debug_clusters").as_bool();
  
  clusterer_ = std::make_unique<Clusterer>(shared_from_this(), cluster_config);
  
  // validator parameters
  validator::Config validator_config;
  declare_parameter("expected_radius", 0.028);
  declare_parameter("radius_tolerance", 0.008);
  declare_parameter("publish_debug_validation", false);
  
  validator_config.expected_radius = get_parameter("expected_radius").as_double();
  validator_config.radius_tolerance = get_parameter("radius_tolerance").as_double();
  validator_config.publish_debug = get_parameter("publish_debug_validation").as_bool();
  
  validator_ = std::make_unique<validator>(shared_from_this(), validator_config);
  
  // Tracker parameters
  Tracker::Config tracker_config;
  declare_parameter("max_tracks", 6);
  declare_parameter("association_distance", 0.15);
  declare_parameter("max_invisible_frames", 20);
  declare_parameter("publish_debug_tracks", false);
  
  tracker_config.max_tracks = get_parameter("max_tracks").as_int();
  tracker_config.association_distance = get_parameter("association_distance").as_double();
  tracker_config.max_invisible_frames = get_parameter("max_invisible_frames").as_int();
  tracker_config.publish_debug_tracks = get_parameter("publish_debug_tracks").as_bool();
  
  tracker_ = std::make_unique<Tracker>(shared_from_this(), tracker_config);
  
  // Pattern Matcher parameters
  declare_parameter("enable_pattern_matching", true);
  declare_parameter("publish_debug_pattern", false);
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