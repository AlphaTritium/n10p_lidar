// ============================================================================
// POLE DETECTION SYSTEM - MULTI-OBJECT TRACKER
// ============================================================================
// 
// This file implements the multi-object tracking system for pole detection:
// - Track association and data association logic
// - EMA smoothing for stable position estimation
// - Confidence-based track management
// - Stale track removal and lifecycle management
// - Debug visualization for track states
//
// ============================================================================

#include "tracker.hpp"
#include <algorithm>
#include <cmath>

namespace pole_detection
{

// ============================================================================
// SECTION 1: TRACKER INITIALIZATION AND CONFIGURATION
// ============================================================================
// Constructor sets up debug publishing and initializes tracking parameters
// ============================================================================

Tracker::Tracker(rclcpp::Node::SharedPtr node, const Config& config)
  : node_(node), config_(config), next_track_id_(0)
{
  if (config_.publish_debug_tracks) {
    debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/debug/tracks", 10);
    RCLCPP_INFO(node_->get_logger(), "Tracker debug publishing ENABLED");
  }
}

// ============================================================================
// SECTION 2: MAIN TRACK UPDATE LOOP - DETECTION ASSOCIATION
// ============================================================================
// Core tracking logic: association, update, creation, and stale track removal
// ============================================================================

std::vector<TrackedPole> Tracker::update(
  const std::vector<PoleCandidate>& detections,
  const std_msgs::msg::Header& header)
{
  auto current_time = node_->now();
  
  // ==========================================================================
  // STEP 2.1: MARK EXISTING TRACKS AS POTENTIALLY INVISIBLE
  // ==========================================================================
  // Prepare for new frame by marking all tracks as potentially invisible
  // ==========================================================================
  
  for (auto& track : tracks_) {
    track.markInvisible();
  }
  
  // ==========================================================================
  // STEP 2.2: ASSOCIATE DETECTIONS WITH EXISTING TRACKS
  // Data association using nearest neighbor within association distance
  // ==========================================================================
  
  std::vector<bool> detection_matched(detections.size(), false);
  
  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& detection = detections[i];
    
    // Find best matching track using distance-based association
    int best_track_idx = -1;
    double best_distance = config_.association_distance;
    
    for (size_t j = 0; j < tracks_.size(); ++j) {
      if (!tracks_[j].is_active) continue;
      
      double dx = detection.centroid.x - tracks_[j].position.x;
      double dy = detection.centroid.y - tracks_[j].position.y;
      double distance = std::hypot(dx, dy);
      
      if (distance < best_distance) {
        best_distance = distance;
        best_track_idx = j;
      }
    }
    
    // ========================================================================
    // STEP 2.3: UPDATE EXISTING TRACK OR CREATE NEW TRACK
    // Update matched tracks or create new tracks within maximum limits
    // ========================================================================
    
    if (best_track_idx >= 0) {
      double confidence = 1.0;  // Default confidence
      if (!detection.rejection_reason.empty()) {
        confidence = 0.5;
      }
      
      // Use new parameters from rc2026_head_finder
      tracks_[best_track_idx].update(detection.centroid, confidence, current_time, 
                                     config_.confirmation_threshold, 
                                     config_.ema_alpha, 
                                     config_.max_jump_distance);
      detection_matched[i] = true;
      
      RCLCPP_DEBUG(node_->get_logger(),
        "Track %d UPDATED: pos=(%.3f, %.3f), detections=%d",
        tracks_[best_track_idx].track_id,
        tracks_[best_track_idx].position.x,
        tracks_[best_track_idx].position.y,
        tracks_[best_track_idx].detection_count);
    } else {
      // Create new track if under limit
      if (static_cast<int>(tracks_.size()) < config_.max_tracks) {
        double confidence = 1.0;
        if (!detection.rejection_reason.empty()) {
          confidence = 0.5;
        }
        
        TrackedPole new_track(next_track_id_++, detection.centroid, confidence, current_time);
        tracks_.push_back(new_track);
        
        RCLCPP_INFO(node_->get_logger(),
          "NEW Track %d created at (%.3f, %.3f)",
          new_track.track_id, detection.centroid.x, detection.centroid.y);
      }
    }
  }
  
  // ==========================================================================
  // STEP 2.4: REMOVE STALE TRACKS
  // Clean up tracks that have been invisible for too many frames
  // ==========================================================================
  
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
      [this](const TrackedPole& track) {
        bool stale = track.isStale(config_.max_invisible_frames);
        if (stale) {
          RCLCPP_DEBUG(node_->get_logger(),
            "Track %d REMOVED (invisible for %d frames)",
            track.track_id, track.invisible_count);
        }
        return stale;
      }),
    tracks_.end());
  
  // ==========================================================================
  // STEP 2.5: PUBLISH DEBUG VISUALIZATION
  // Generate debug markers for visualization if enabled
  // ==========================================================================
  
  if (config_.publish_debug_tracks && debug_pub_) {
    publishDebugMarkers(tracks_, header);
  }
  
  return tracks_;
}

// ============================================================================
// SECTION 3: DEBUG VISUALIZATION MARKER GENERATION
// Creates visualization markers for track visualization in RViz
// ============================================================================

void Tracker::publishDebugMarkers(
  const std::vector<TrackedPole>& tracks,
  const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.ns = "tracked_poles";
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  
  for (size_t i = 0; i < tracks.size(); ++i) {
    const auto& track = tracks[i];
    
    // Track color based on confirmation status
    float r, g, b;
    if (track.is_confirmed) {
      r = 0.0; g = 1.0; b = 0.0;  // Green for confirmed
    } else {
      r = 1.0; g = 1.0; b = 0.0;  // Yellow for tentative
    }
    
    // Sphere marker at track position
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header = header;
    sphere_marker.ns = "tracked_poles";
    sphere_marker.id = i * 4;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.pose.position = track.position;
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = track.avg_features_confidence * 0.15;
    sphere_marker.scale.y = track.avg_features_confidence * 0.15;
    sphere_marker.scale.z = track.avg_features_confidence * 0.15;
    sphere_marker.color.r = r;
    sphere_marker.color.g = g;
    sphere_marker.color.b = b;
    sphere_marker.color.a = 0.8;
    sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    // Track ID and detection count label
    visualization_msgs::msg::Marker id_marker;
    id_marker.header = header;
    id_marker.ns = "track_ids";
    id_marker.id = i * 4 + 1;
    id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    id_marker.action = visualization_msgs::msg::Marker::ADD;
    id_marker.pose.position.x = track.position.x;
    id_marker.pose.position.y = track.position.y;
    id_marker.pose.position.z = track.position.z + 0.3;
    id_marker.pose.orientation.w = 1.0;
    id_marker.scale.z = 0.12;
    id_marker.color.r = 1.0;
    id_marker.color.g = 1.0;
    id_marker.color.b = 1.0;
    id_marker.color.a = 1.0;
    
    // Enhanced text label with action feedback info
    char text_buf[128];
    snprintf(text_buf, sizeof(text_buf), "T%d\nDets:%d\nY:%.3f", 
             track.track_id, track.detection_count, track.position.y);
    id_marker.text = text_buf;
    id_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    // Path history visualization
    visualization_msgs::msg::Marker path_marker;
    path_marker.header = header;
    path_marker.ns = "track_paths";
    path_marker.id = i * 4 + 2;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.01;
    path_marker.color.r = r;
    path_marker.color.g = g;
    path_marker.color.b = b;
    path_marker.color.a = 0.5;
    path_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    
    geometry_msgs::msg::Point p;
    p.x = track.position.x;
    p.y = track.position.y;
    p.z = track.position.z;
    path_marker.points.push_back(p);
    
    // Action feedback marker (like rc2026_head_finder)
    visualization_msgs::msg::Marker action_marker;
    action_marker.header = header;
    action_marker.ns = "action_feedback";
    action_marker.id = i * 4 + 3;
    action_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    action_marker.action = visualization_msgs::msg::Marker::ADD;
    action_marker.pose.position.x = track.position.x;
    action_marker.pose.position.y = track.position.y;
    action_marker.pose.position.z = track.position.z + 0.5;
    action_marker.pose.orientation.w = 1.0;
    action_marker.scale.z = 0.1;
    action_marker.color.r = 0.0f;
    action_marker.color.g = 0.5f;
    action_marker.color.b = 1.0f;
    action_marker.color.a = 1.0f;
    
    char action_buf[128];
    snprintf(action_buf, sizeof(action_buf), "Action Feedback\nY Offset: %.3f m", track.position.y);
    action_marker.text = action_buf;
    action_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    marker_array.markers.push_back(sphere_marker);
    marker_array.markers.push_back(id_marker);
    marker_array.markers.push_back(path_marker);
    marker_array.markers.push_back(action_marker);
  }
  
  debug_pub_->publish(marker_array);
}

}  // namespace pole_detection
