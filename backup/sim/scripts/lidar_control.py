#!/usr/bin/env python3
"""
LiDAR System Controller
Turn the entire LiDAR processing system ON/OFF to save resources
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import time

class LidarController(Node):
    def __init__(self):
        super().__init__('lidar_controller')
        self.enable_pub = self.create_publisher(Bool, '/lidar_processor/enable', 10)
        self.status_sub = self.create_subscription(Bool, '/lidar_processor/status', self.status_callback, 10)
        self.current_status = None
        self.status_received = False
        
    def status_callback(self, msg):
        self.current_status = msg.data
        self.status_received = True
        
    def set_system_state(self, enable):
        msg = Bool()
        msg.data = enable
        self.enable_pub.publish(msg)
        state = "ON" if enable else "OFF"
        print(f"✅ Sent command to turn system {state}")
        
def main():
    if len(sys.argv) < 2:
        print("\n=== LiDAR System Controller ===")
        print("Usage: ros2 run sim lidar_control.py [on|off|status]")
        print("\nCommands:")
        print("  on     - Turn LiDAR processing ON (full operation)")
        print("  off    - Turn LiDAR processing OFF (save resources)")
        print("  status - Check current system status")
        print("\nExamples:")
        print("  ros2 run sim lidar_control.py on")
        print("  ros2 run sim lidar_control.py off")
        print("  ros2 run sim lidar_control.py status")
        sys.exit(1)
    
    command = sys.argv[1].lower()
    
    rclpy.init()
    controller = LidarController()
    
    # Wait for publisher to be ready
    time.sleep(0.5)
    
    # Spin a few times to get current status
    for _ in range(5):
        rclpy.spin_once(controller, timeout_sec=0.2)
    
    if command == 'on':
        controller.set_system_state(True)
        time.sleep(0.5)
        rclpy.spin_once(controller, timeout_sec=0.5)
        if controller.status_received:
            state = "RUNNING ✅" if controller.current_status else "STOPPED ⏸️"
            print(f"📊 Current Status: {state}")
        else:
            print("⚠️  Command sent, waiting for confirmation...")
            
    elif command == 'off':
        controller.set_system_state(False)
        time.sleep(0.5)
        rclpy.spin_once(controller, timeout_sec=0.5)
        if controller.status_received:
            state = "RUNNING ✅" if controller.current_status else "STOPPED ⏸️"
            print(f"📊 Current Status: {state}")
        else:
            print("⚠️  Command sent, waiting for confirmation...")
            
    elif command == 'status':
        if controller.status_received and controller.current_status is not None:
            state = "RUNNING ✅" if controller.current_status else "STOPPED ⏸️"
            print(f"📊 LiDAR System Status: {state}")
            if controller.current_status:
                print("   💡 Processing is ACTIVE - using full CPU")
            else:
                print("   💤 Processing is PAUSED - saving resources (~95% CPU saved)")
        else:
            print("⚠️  Could not determine status (no response from node)")
            print("   Make sure lidar_pointcloud_processor is running!")
    else:
        print(f"❌ Unknown command: {command}")
        sys.exit(1)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()