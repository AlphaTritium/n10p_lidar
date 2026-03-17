#include "pole_detection_node.hpp"
#include "pattern_matcher.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace pole_detection
{

PoleDetectionNode::PoleDetectionNode()
  : Node("pole_detection"), modules_initialized_(false)
{
  // Declare parameters
  declare_parameter("range_min", 0.2);
  declare_parameter("range_max", 0.8);
  declare_parameter("z_min", -0.3);
  declare_parameter("z_max", 0.3);
  declare_parameter("voxel_leaf_size", 0.01);
  declare_parameter("use_intensity_filter", true);
  declare_parameter("min_intensity", 50.0);
  // declare_parameter("publish_debug_cloud", false);
  declare_parameter("cluster_tolerance", 0.05);
  declare_parameter("cluster_min_size", 6);
  declare_parameter("cluster_max_size", 100);
  declare_parameter("publish_debug_clusters", false);
  declare_parameter("expected_radius", 0.028);
  declare_parameter("radius_tolerance", 0.008);
  declare_parameter("publish_debug_validation", false);
  declare_parameter("max_tracks", 6);
  declare_parameter("association_distance", 0.15);
  declare_parameter("max_invisible_frames", 20);
  declare_parameter("publish_debug_tracks", false);
  declare_parameter("enable_pattern_matching", true);
  declare_parameter("publish_debug_pattern", false);
  
  // Create subscribers and publishers
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lslidar_point_cloud", 10,
    std::bind(&PoleDetectionNode::cloudCallback, this, std::placeholders::_1));
  
  objects_pub_ = create_publisher<lslidar_msgs::msg::DetectedObjects>(
    "/detected_objects", 10);
  poles_pub_ = create_publisher<lslidar_msgs::msg::DetectedObjects>(
    "/detected_poles", 10);
  
  RCLCPP_INFO(get_logger(), "Pole Detection Node initialized");
  RCLCPP_INFO(get_logger(), "Pipeline: Preprocessor → Clusterer → validator → Tracker → PatternMatcher");
  RCLCPP_INFO(get_logger(), "Modules will be initialized on first point cloud");
}

void PoleDetectionNode::ensureModulesInitialized()
{
  if (modules_initialized_) {
    return;
  }
  
  initializeModules();
  modules_initialized_ = true;
  RCLCPP_INFO(get_logger(), "All processing modules now initialized");
}

void PoleDetectionNode::initializeModules()
{
  auto shared_node = shared_from_this();
  
  /**
  Preprocessor::Config preproc_config;
  preproc_config.range_min = get_parameter("range_min").as_double();
  preproc_config.range_max = get_parameter("range_max").as_double();
  preproc_config.z_min = get_parameter("z_min").as_double();
  preproc_config.z_max = get_parameter("z_max").as_double();
  preproc_config.voxel_leaf_size = get_parameter("voxel_leaf_size").as_double();
  preproc_config.use_intensity_filter = get_parameter("use_intensity_filter").as_bool();
  preproc_config.min_intensity = get_parameter("min_intensity").as_double();
  preproc_config.publish_debug_cloud = get_parameter("publish_debug_cloud").as_bool();
  preprocessor_ = std::make_unique<Preprocessor>(shared_node, preproc_config);
  */
  
  Clusterer::Config cluster_config;
  cluster_config.cluster_tolerance = get_parameter("cluster_tolerance").as_double();
  cluster_config.cluster_min_size = get_parameter("cluster_min_size").as_int();
  cluster_config.cluster_max_size = get_parameter("cluster_max_size").as_int();
  cluster_config.publish_debug_markers = get_parameter("publish_debug_clusters").as_bool();
  clusterer_ = std::make_unique<Clusterer>(shared_node, cluster_config);
  
  validator::Config validator_config;
  validator_config.expected_radius = get_parameter("expected_radius").as_double();
  validator_config.radius_tolerance = get_parameter("radius_tolerance").as_double();
  validator_config.publish_debug = get_parameter("publish_debug_validation").as_bool();
  validator_ = std::make_unique<validator>(shared_node, validator_config);
  
  Tracker::Config tracker_config;
  tracker_config.max_tracks = get_parameter("max_tracks").as_int();
  tracker_config.association_distance = get_parameter("association_distance").as_double();
  tracker_config.max_invisible_frames = get_parameter("max_invisible_frames").as_int();
  tracker_config.publish_debug_tracks = get_parameter("publish_debug_tracks").as_bool();
  tracker_ = std::make_unique<Tracker>(shared_node, tracker_config);

  PatternMatcher::Config pattern_config;
  auto distances_param = get_parameter("expected_inter_pole_distances").as_double_array();
  if (distances_param.empty()) {
    pattern_config.expected_distances = {0.185};
  } else {
    pattern_config.expected_distances = std::vector<double>(distances_param.begin(), distances_param.end());
  }
  pattern_config.distance_tolerance = get_parameter("distance_match_tolerance").as_double();
  pattern_config.publish_debug = get_parameter("publish_debug_pattern").as_bool();
  pattern_matcher_ = std::make_unique<PatternMatcher>(shared_node, pattern_config);

  RCLCPP_INFO(get_logger(), "Pole Detection Node initialized");
  RCLCPP_INFO(get_logger(), "Pipeline: Clusterer → Validator → Tracker → PatternMatcher (raw cloud, no preprocessing)");
  RCLCPP_INFO(get_logger(), "Modules will be initialized on first point cloud");
  RCLCPP_INFO(get_logger(), "===========================================");
  RCLCPP_INFO(get_logger(), "Output Topics:");
  RCLCPP_INFO(get_logger(), "  - /detected_poles (final pole positions)");
  RCLCPP_INFO(get_logger(), "  - /detected_objects (alias for backward compatibility)");
  RCLCPP_INFO(get_logger(), "Debug Topics:");
  RCLCPP_INFO(get_logger(), "  - /debug/clusters_raw (orange spheres - ALL candidates)");
  RCLCPP_INFO(get_logger(), "  - /debug/validated_poles (green spheres - passed)");
  RCLCPP_INFO(get_logger(), "  - /debug/rejected_poles (yellow spheres - failed + reasons)");
  RCLCPP_INFO(get_logger(), "  - /debug/tracks (blue/green spheres - tracked poles)");
  RCLCPP_INFO(get_logger(), "  - /debug/pattern_matches (lines - inter-pole distances)");
  RCLCPP_INFO(get_logger(), "===========================================");
}


void PoleDetectionNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  // Lazy initialization - ensures modules are ready before first use
  ensureModulesInitialized();
  
  /** 
  // Stage 1: Preprocessing
  auto preprocessed = preprocessor_->process(msg);
  if (!preprocessed || preprocessed->empty()) {
    RCLCPP_DEBUG(get_logger(), "Preprocessing produced empty cloud");
    return;
  }
  */
  
    // Stage 1: Convert raw point cloud to PCL format (skip preprocessing/filtering)
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  
  try {
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_DEBUG(get_logger(), "Converted raw point cloud: %zu points", cloud->points.size());
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "Point cloud conversion error: %s", e.what());
    RCLCPP_WARN(get_logger(), "Attempting fallback conversion...");
    
    // Fallback: convert as PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_xyz);
    
    cloud->header = cloud_xyz->header;
    cloud->width = cloud_xyz->width;
    cloud->height = cloud_xyz->height;
    cloud->is_dense = cloud_xyz->is_dense;
    cloud->points.resize(cloud_xyz->points.size());
    
    for (size_t i = 0; i < cloud_xyz->points.size(); ++i) {
      cloud->points[i].x = cloud_xyz->points[i].x;
      cloud->points[i].y = cloud_xyz->points[i].y;
      cloud->points[i].z = cloud_xyz->points[i].z;
      cloud->points[i].intensity = 0.0;
    }
  }
  
  // Skip preprocessing - pass raw cloud directly to clustering
  // This ensures no clusters are missed due to filtering
  
  // Stage 2: Clustering (now receives raw unfiltered cloud)
  auto candidates = clusterer_->extractClusters(cloud, msg->header);
  RCLCPP_DEBUG(get_logger(), "Extracted %zu cluster candidates from raw cloud", candidates.size());
  
  // Stage 3: validation
  auto validated = validator_->validate(candidates, cloud);
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
    lslidar_msgs::msg::DetectedObjects objects_msg = poles_msg;
    objects_pub_->publish(objects_msg);
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