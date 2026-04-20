// ============================================================================
// POLE DETECTION SYSTEM - ACTION SERVER FOR POLE TRACKING
// ============================================================================
// 
// This file implements the ROS2 Action Server for pole tracking tasks:
// - Behavior tree compatible action server interface
// - Multi-threaded task execution with cancellation support
// - Real-time feedback publishing for pole detection monitoring
// - Timeout handling and graceful error recovery
// - Detection processing for pole positioning and tracking
//
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pole_detection/msg/detected_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pole_detection/action/track_poles.hpp>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

// ACTION SERVER CLASS DEFINITION
// Main class implementing the pole tracking action server

class TrackPolesActionServer : public rclcpp::Node
{
public:
  using TrackPoles = pole_detection::action::TrackPoles;
  using GoalHandleTrackPoles = rclcpp_action::ServerGoalHandle<TrackPoles>;
  
  // SECTION 1.1: CONSTRUCTOR AND INITIALIZATION
  // Sets up subscriptions, action server, and multi-threading components
  
  explicit TrackPolesActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("track_poles_action_server", options),
      is_running_(false),
      should_cancel_(false)
  {
    // Subscribe to detected objects for pole position data
    objects_sub_ = this->create_subscription<pole_detection::msg::DetectedObjects>(
      "/detected_objects", 10,
      std::bind(&TrackPolesActionServer::objectsCallback, this, std::placeholders::_1));
    
    // Create Action Server - compatible with behavior trees
    action_server_ = rclcpp_action::create_server<TrackPoles>(
      this,
      "/track_poles",
      std::bind(&TrackPolesActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrackPolesActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&TrackPolesActionServer::handleAccepted, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "✅ TrackPoles Action Server 初始化完成 (行为树兼容)");
  }

  virtual ~TrackPolesActionServer()
  {
    cleanup();
  }

private:
  // SECTION 1.2: MEMBER VARIABLES AND COMPONENTS
  // Multi-thread safe variables and ROS2 components
  
  rclcpp::Subscription<pole_detection::msg::DetectedObjects>::SharedPtr objects_sub_;
  rclcpp_action::Server<TrackPoles>::SharedPtr action_server_;
  
  // Multi-threading safe variables
  std::atomic<bool> is_running_;
  std::atomic<bool> should_cancel_;
  std::shared_ptr<GoalHandleTrackPoles> current_goal_handle_;
  std::mutex goal_mutex_;
  
  pole_detection::msg::DetectedObjects::SharedPtr latest_detections_;
  std::mutex detection_mutex_;
  
  // ==========================================================================
  // SECTION 2: DETECTION CALLBACK AND DATA MANAGEMENT
  // Processes incoming detection data and maintains latest state
  // ==========================================================================
  
  void objectsCallback(const pole_detection::msg::DetectedObjects::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    latest_detections_ = msg;
  }
  
  // ==========================================================================
  // SECTION 3: ACTION SERVER CALLBACK HANDLERS
  // ==========================================================================
  // Handles goal requests, cancellations, and task acceptance
  // ==========================================================================
  
  // 3.1: Cancel request handler (preemptability)
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleTrackPoles> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    
    // Set cancellation flag for graceful termination
    should_cancel_.store(true);
    
    // Immediate response to cancellation
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  // 3.2: Goal request handler
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const TrackPoles::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), 
      "收到杆柱跟踪任务请求, start_tracking=%s", 
      goal->start_tracking ? "true" : "false");
    
    (void)uuid;
    
    // Check if another task is already running
    if (is_running_.load()) {
      RCLCPP_WARN(this->get_logger(), "已有任务正在运行，拒绝新目标");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  // 3.3: Goal acceptance handler
  void handleAccepted(const std::shared_ptr<GoalHandleTrackPoles> goal_handle)
  {
    // Reset state for new task execution
    should_cancel_.store(false);
    is_running_.store(true);
    
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      current_goal_handle_ = goal_handle;
    }
    
    RCLCPP_INFO(this->get_logger(), "开始执行杆柱跟踪任务");
    
    // 3.4: Use multi-threading: avoid blocking ROS callback queue
    std::thread{std::bind(&TrackPolesActionServer::executeTask, this, std::placeholders::_1), goal_handle}.detach();
  }
  
  // ==========================================================================
  // SECTION 4: MAIN TASK EXECUTION LOOP
  // ==========================================================================
  // Core task execution with cancellation, timeout, and feedback handling
  // ==========================================================================
  
  void executeTask(std::shared_ptr<GoalHandleTrackPoles> goal_handle)
  {
    auto result = std::make_shared<TrackPoles::Result>();
    const auto goal = goal_handle->get_goal();
    
    // Task execution frequency (30Hz)
    rclcpp::Rate loop_rate(30);
    
    // Timeout settings (10 seconds)
    const auto max_duration = std::chrono::seconds(10);
    const auto start_time = std::chrono::steady_clock::now();
    
    // Last feedback time tracking
    auto last_feedback_time = start_time;
    
    while (rclcpp::ok() && is_running_.load()) {
      // ======================================================================
      // STEP 4.1: CHECK FOR CANCELLATION REQUEST (PREEMPTABILITY)
      // ======================================================================
      // Gracefully handle task cancellation requests
      // ======================================================================
      
      if (should_cancel_.load() || goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "任务被取消");
        result->success = false;
        
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          goal_handle->canceled(result);
        }
        
        cleanup();
        return;
      }
      
      // ======================================================================
      // STEP 4.2: PUBLISH STATUS FEEDBACK (REGULAR INTERVALS)
      // ======================================================================
      // Provide regular feedback for task monitoring
      // ======================================================================
      
      auto current_time = std::chrono::steady_clock::now();
      if (current_time - last_feedback_time > std::chrono::milliseconds(100)) {
        publishFeedback(goal_handle);
        last_feedback_time = current_time;
      }
      
      // ======================================================================
      // STEP 4.3: CHECK FOR TIMEOUT CONDITIONS
      // ======================================================================
      // Prevent infinite execution with timeout protection
      // ======================================================================
      
      if (current_time - start_time > max_duration) {
        RCLCPP_WARN(this->get_logger(), "任务超时");
        result->success = false;
        
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          goal_handle->abort(result);
        }
        
        cleanup();
        return;
      }
      
      // ======================================================================
      // STEP 4.4: PROCESS DETECTION DATA FOR POLE TRACKING
      // ======================================================================
      // Core detection processing logic
      // ======================================================================
      
      if (processDetection(goal)) {
        RCLCPP_INFO(this->get_logger(), "杆柱跟踪成功完成");
        result->success = true;
        
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          goal_handle->succeed(result);
        }
        
        cleanup();
        return;
      }
      
      loop_rate.sleep();
    }
    
    // Normal exit path
    cleanup();
  }
  
  // ==========================================================================
  // SECTION 5: DETECTION PROCESSING LOGIC
  // ==========================================================================
  // Core algorithm for pole detection and tracking
  // ==========================================================================
  
  bool processDetection(std::shared_ptr<const TrackPoles::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    
    if (!latest_detections_ || latest_detections_->objects.empty()) {
      return false;
    }
    
    // Check if we have detected at least one pole
    if (latest_detections_->objects.size() > 0) {
      RCLCPP_DEBUG(this->get_logger(),
        "检测到 %zu 个杆柱对象", latest_detections_->objects.size());
      
      // For continuous tracking, we can return true immediately
      // Or implement specific logic based on goal->start_tracking
      return true;
    }
    
    return false;
  }
  
  // ==========================================================================
  // SECTION 6: FEEDBACK PUBLISHING AND STATE MANAGEMENT
  // ==========================================================================
  // Feedback generation and system cleanup functions
  // ==========================================================================
  
  void publishFeedback(std::shared_ptr<GoalHandleTrackPoles> goal_handle)
  {
    auto feedback = std::make_shared<TrackPoles::Feedback>();
    
    {
      std::lock_guard<std::mutex> lock(detection_mutex_);
      
      // Clear previous data
      feedback->detected_poles_count = 0;
      feedback->pole_positions.clear();
      feedback->pole_distances_x.clear();
      feedback->pole_distances_y.clear();
      feedback->pole_confidences.clear();
      
      if (latest_detections_ && !latest_detections_->objects.empty()) {
        feedback->detected_poles_count = static_cast<int32_t>(latest_detections_->objects.size());
        
        // Populate feedback with detected pole data
        for (const auto& obj : latest_detections_->objects) {
          geometry_msgs::msg::Point point;
          point.x = obj.x;
          point.y = obj.y;
          point.z = obj.z;
          
          feedback->pole_positions.push_back(point);
          feedback->pole_distances_x.push_back(static_cast<float>(obj.x));
          feedback->pole_distances_y.push_back(static_cast<float>(obj.y));
          feedback->pole_confidences.push_back(1.0f); // Default confidence
        }
      }
    }
    
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_handle->is_active()) {
        goal_handle->publish_feedback(feedback);
      }
    }
  }
  
  void cleanup()
  {
    is_running_.store(false);
    should_cancel_.store(false);
    
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      current_goal_handle_.reset();
    }
  }
};

// ============================================================================
// SECTION 7: MAIN APPLICATION ENTRY POINT
// ============================================================================
// ROS2 node initialization and multi-threaded executor setup
// ============================================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // 创建节点选项，确保多线程安全
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false); // 避免内部进程通信问题
  
  auto node = std::make_shared<TrackPolesActionServer>(options);
  
  // 使用多线程执行器确保响应性
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  try {
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("track_poles"), "执行器异常: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}