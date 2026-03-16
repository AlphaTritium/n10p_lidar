#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rc2026_interfaces/action/gripper_control.hpp>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include <vector>
#include <string>
#include <thread>

class GripperControlActionServer : public rclcpp::Node
{
public:
  using GripperControl = rc2026_interfaces::action::GripperControl;
  using GoalHandleGripperControl = rclcpp_action::ServerGoalHandle<GripperControl>;
  
  explicit GripperControlActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("gripper_control_action_server", options)
  {
    objects_sub_ = this->create_subscription<lslidar_msgs::msg::DetectedObjects>(
      "/detected_objects", 10,
      std::bind(&GripperControlActionServer::objectsCallback, this, std::placeholders::_1));
    
    action_server_ = rclcpp_action::create_server<GripperControl>(
      this,
      "/task/gripper_control",
      std::bind(&GripperControlActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GripperControlActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&GripperControlActionServer::handleAccepted, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "GripperControl Action Server initialized");
  }

private:
  rclcpp::Subscription<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_sub_;
  rclcpp_action::Server<GripperControl>::SharedPtr action_server_;
  
  lslidar_msgs::msg::DetectedObjects::SharedPtr latest_detections_;
  std::shared_ptr<GoalHandleGripperControl> current_goal_handle_;
  bool task_running_ = false;
  
  void objectsCallback(const lslidar_msgs::msg::DetectedObjects::SharedPtr msg)
  {
    latest_detections_ = msg;
    
    if (current_goal_handle_) {
      auto feedback = std::make_shared<GripperControl::Feedback>();
      feedback->task_state = !msg->objects.empty();
      current_goal_handle_->publish_feedback(feedback);
    }
  }
  
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const GripperControl::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void handleAccepted(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    current_goal_handle_ = goal_handle;
    task_running_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Goal accepted - Starting positioning task");
    std::thread{std::bind(&GripperControlActionServer::executeTask, this, std::placeholders::_1), goal_handle}.detach();
  }
  
  void executeTask(std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    auto result = std::make_shared<GripperControl::Result>();
    const int max_iterations = 50;
    int iteration = 0;
    
    while (rclcpp::ok() && iteration < max_iterations) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        task_running_ = false;
        RCLCPP_INFO(this->get_logger(), "Task canceled");
        return;
      }
      
      if (latest_detections_ && !latest_detections_->objects.empty()) {
        RCLCPP_INFO(this->get_logger(),
          "✓ SUCCESS: %zu poles detected", latest_detections_->objects.size());
        
        result->success = true;
        goal_handle->succeed(result);
        task_running_ = false;
        return;
      }
      
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      iteration++;
    }
    
    RCLCPP_WARN(this->get_logger(), "⚠ TIMEOUT: No poles detected after %d iterations", max_iterations);
    result->success = false;
    goal_handle->abort(result);
    task_running_ = false;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperControlActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}