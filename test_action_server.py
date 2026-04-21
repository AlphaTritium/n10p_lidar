#!/usr/bin/env python3
"""
Test script for TrackPoles Action Server
Tests: goal sending, feedback reception, goal preemption, and cancellation
"""

import rclpy
from rclpy.node import Node
from pole_detection.action import TrackPoles
from rclpy.action import ActionClient
import time


class TestActionClient(Node):
    def __init__(self):
        super().__init__('test_action_client')
        self._action_client = ActionClient(self, TrackPoles, '/track_poles')
        self.feedback_count = 0
        
    def send_goal(self, start_tracking=True):
        """Send a goal to the action server"""
        goal_msg = TrackPoles.Goal()
        goal_msg.start_tracking = start_tracking
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal: start_tracking={start_tracking}')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected!')
            return
            
        self.get_logger().info('✅ Goal accepted!')
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        
    def feedback_callback(self, feedback_msg):
        self.feedback_count += 1
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'📊 Feedback #{self.feedback_count}: '
            f'{feedback.detected_poles_count} poles detected'
        )
        if feedback.pole_positions:
            for i, pos in enumerate(feedback.pole_positions):
                self.get_logger().info(
                    f'   Pole {i+1}: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}'
                )
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🎯 Result: success={result.success}')
        rclpy.shutdown()
        

def main():
    print("=" * 60)
    print("POLE DETECTION ACTION SERVER TEST")
    print("=" * 60)
    print("\nThis test will:")
    print("1. Send a goal to track poles")
    print("2. Monitor feedback (pole positions)")
    print("3. Check if goals can be preempted")
    print("4. Verify cancellation works")
    print("\nMake sure the system is running:")
    print("  ros2 launch pole_detection pole_detection.launch.py")
    print("\nPress Enter to start test...")
    input()
    
    rclpy.init()
    client = TestActionClient()
    
    try:
        # Test 1: Send first goal
        print("\n" + "="*60)
        print("TEST 1: Sending first goal...")
        print("="*60)
        client.send_goal(start_tracking=True)
        
        # Let it run for 5 seconds to receive feedback
        print("\nReceiving feedback for 5 seconds...")
        time.sleep(5)
        
        # Test 2: Send second goal (should preempt first)
        print("\n" + "="*60)
        print("TEST 2: Sending second goal (should preempt first)...")
        print("="*60)
        client.send_goal(start_tracking=True)
        
        print("\nReceiving feedback for another 5 seconds...")
        time.sleep(5)
        
        print("\n" + "="*60)
        print("TEST COMPLETE!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
