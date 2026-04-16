#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rc2026_interfaces/action/gripper_control.hpp>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

class GripperControlActionServer : public rclcpp::Node
{
public:
  using GripperControl = rc2026_interfaces::action::GripperControl;
  using GoalHandleGripperControl = rclcpp_action::ServerGoalHandle<GripperControl>;
  
  explicit GripperControlActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("gripper_control_action_server", options),
      is_running_(false),
      should_cancel_(false)
  {
    // 订阅检测到的物体
    objects_sub_ = this->create_subscription<lslidar_msgs::msg::DetectedObjects>(
      "/detected_objects", 10,
      std::bind(&GripperControlActionServer::objectsCallback, this, std::placeholders::_1));
    
    // 创建Action Server - 符合行为树要求
    action_server_ = rclcpp_action::create_server<GripperControl>(
      this,
      "/task/gripper_control",
      std::bind(&GripperControlActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GripperControlActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&GripperControlActionServer::handleAccepted, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "✅ GripperControl Action Server 初始化完成 (行为树兼容)");
  }

  virtual ~GripperControlActionServer()
  {
    cleanup();
  }

private:
  rclcpp::Subscription<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_sub_;
  rclcpp_action::Server<GripperControl>::SharedPtr action_server_;
  
  // 多线程安全变量
  std::atomic<bool> is_running_;
  std::atomic<bool> should_cancel_;
  std::shared_ptr<GoalHandleGripperControl> current_goal_handle_;
  std::mutex goal_mutex_;
  
  lslidar_msgs::msg::DetectedObjects::SharedPtr latest_detections_;
  std::mutex detection_mutex_;
  
  void objectsCallback(const lslidar_msgs::msg::DetectedObjects::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    latest_detections_ = msg;
  }
  
  // 1. 可抢占性: 响应 cancel_goal 请求
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    
    // 设置取消标志
    should_cancel_.store(true);
    
    // 立即响应取消
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const GripperControl::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), 
      "收到长杆定位任务请求, 目标位置: X=%.1fmm, Y=%.1fmm", 
      goal->x_error, goal->y_error);
    
    (void)uuid;
    
    // 检查是否已有任务运行
    if (is_running_.load()) {
      RCLCPP_WARN(this->get_logger(), "已有任务正在运行，拒绝新目标");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  void handleAccepted(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    // 重置状态
    should_cancel_.store(false);
    is_running_.store(true);
    
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      current_goal_handle_ = goal_handle;
    }
    
    RCLCPP_INFO(this->get_logger(), "开始执行长杆定位任务");
    
    // 3. 使用多线程: 不阻塞ROS回调队列
    std::thread{std::bind(&GripperControlActionServer::executeTask, this, std::placeholders::_1), goal_handle}.detach();
  }
  
  void executeTask(std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    auto result = std::make_shared<GripperControl::Result>();
    const auto goal = goal_handle->get_goal();
    
    // 任务执行频率 (30Hz)
    rclcpp::Rate loop_rate(30);
    
    // 超时设置 (10秒)
    const auto max_duration = std::chrono::seconds(10);
    const auto start_time = std::chrono::steady_clock::now();
    
    // 上次反馈时间
    auto last_feedback_time = start_time;
    
    while (rclcpp::ok() && is_running_.load()) {
      // 1. 检查取消请求 (可抢占性)
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
      
      // 2. 发布状态反馈 (定期发布)
      auto current_time = std::chrono::steady_clock::now();
      if (current_time - last_feedback_time > std::chrono::milliseconds(100)) {
        publishFeedback(goal_handle);
        last_feedback_time = current_time;
      }
      
      // 检查超时
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
      
      // 处理检测数据
      if (processDetection(goal)) {
        RCLCPP_INFO(this->get_logger(), "长杆定位成功完成");
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
    
    // 正常退出
    cleanup();
  }
  
  bool processDetection(std::shared_ptr<const GripperControl::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    
    if (!latest_detections_ || latest_detections_->objects.empty()) {
      return false;
    }
    
    // 获取第一个检测到的目标
    auto& first_object = latest_detections_->objects[0];
    float detected_x = static_cast<float>(first_object.x * 1000.0);
    float detected_y = static_cast<float>(first_object.y * 1000.0);
    
    // 计算位置偏差
    float x_diff = std::abs(detected_x - goal->x_error);
    float y_diff = std::abs(detected_y - goal->y_error);
    
    // 位置匹配 (容差±50mm)
    if (x_diff < 50.0f && y_diff < 50.0f) {
      RCLCPP_INFO(this->get_logger(),
        "📍 位置匹配成功: 检测位置 X=%.1fmm, Y=%.1fmm (偏差 ΔX=%.1fmm, ΔY=%.1fmm)",
        detected_x, detected_y, x_diff, y_diff);
      
      return true;
    }
    
    RCLCPP_DEBUG(this->get_logger(),
      "位置检测中: 检测 X=%.1fmm, Y=%.1fmm, 目标 X=%.1fmm, Y=%.1fmm",
      detected_x, detected_y, goal->x_error, goal->y_error);
    
    return false;
  }
  
  void publishFeedback(std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    auto feedback = std::make_shared<GripperControl::Feedback>();
    
    {
      std::lock_guard<std::mutex> lock(detection_mutex_);
      feedback->task_state = (latest_detections_ && !latest_detections_->objects.empty());
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // 创建节点选项，确保多线程安全
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false); // 避免内部进程通信问题
  
  auto node = std::make_shared<GripperControlActionServer>(options);
  
  // 使用多线程执行器确保响应性
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  try {
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("gripper_control"), "执行器异常: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}