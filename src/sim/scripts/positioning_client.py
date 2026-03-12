#!/usr/bin/env python3
"""
Action Client for Object Positioning System
Sends goal to /task/gripper_control action server
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rc2026_interfaces.action import GripperControl
import sys

class PositioningClient(Node):
    def __init__(self):
        super().__init__('positioning_client')
        self._action_client = ActionClient(self, GripperControl, '/task/gripper_control')
        
    def send_goal(self):
        """Send goal to start positioning"""
        self.get_logger().info('⚡ Waiting for action server...')
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Action server not available!')
            return False
        
        self.get_logger().info('✅ Action server connected!')
        
        # Create goal
        goal_msg = GripperControl.Goal()
        goal_msg.task_complete = True  # Start the task
        
        self.get_logger().info('🎯 Sending goal: START POSITIONING')
        
        # Send goal asynchronously
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by server')
            return
        
        self.get_logger().info('✅ Goal accepted! Starting positioning...')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Receive feedback during execution"""
        state = "ALIGNING 🔍" if not feedback_msg.feedback.task_state else "FINISHED ✅"
        self.get_logger().info(f'📊 Feedback: {state}')
    
    def result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        
        if result.success:
            self.get_logger().info('🎉 SUCCESS! Object positioned!')
        else:
            self.get_logger().error('❌ FAILED! Positioning unsuccessful')
        
        # Shutdown after completion
        rclpy.shutdown()

def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'start':
        should_start = True
    else:
        print("\n=== Object Positioning Client ===")
        print("Usage: ros2 run sim positioning_client.py [start]")
        print("\nCommands:")
        print("  start - Send goal to begin object positioning")
        print("\nExample:")
        print("  ros2 run sim positioning_client.py start")
        sys.exit(0)
    
    rclpy.init()
    
    client = PositioningClient()
    
    if client.send_goal():
        rclpy.spin(client)
    else:
        rclpy.shutdown()

if __name__ == '__main__':
    main()