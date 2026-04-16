#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rc2026_interfaces.action import GripperControl
import threading
import time

class GripperControlTester(Node):
    def __init__(self):
        super().__init__('gripper_control_tester')
        self.action_client = ActionClient(self, GripperControl, '/task/gripper_control')
        
    def send_goal(self, x_error, y_error):
        self.get_logger().info(f'发送目标: X={x_error}mm, Y={y_error}mm')
        
        goal_msg = GripperControl.Goal()
        goal_msg.x_error = float(x_error)
        goal_msg.y_error = float(y_error)
        
        self.action_client.wait_for_server()
        
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return
            
        self.get_logger().info('目标已接受')
        self.goal_handle = goal_handle
        
        # 启动取消线程
        cancel_thread = threading.Thread(target=self.cancel_after_delay, args=(goal_handle,))
        cancel_thread.start()
        
        # 获取结果
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        
    def cancel_after_delay(self, goal_handle):
        time.sleep(3)  # 等待3秒后取消
        self.get_logger().info('发送取消请求...')
        
        future = goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done_callback)
        
    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response:
            self.get_logger().info(f'取消响应: {cancel_response}')
        else:
            self.get_logger().info('取消失败')
            
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'收到反馈: task_state={feedback.task_state}')
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'任务结果: success={result.success}')
        

def main():
    rclpy.init()
    
    tester = GripperControlTester()
    
    # 发送测试目标
    tester.send_goal(600.0, 0.0)
    
    # 保持运行
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()