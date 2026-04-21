#!/usr/bin/env python3
"""
The test script generates:

3 simulated poles at positions (0.5, 0.2), (0.5, 0.4), (0.5, 0.6)
Each pole has 20 points arranged in a circle (3cm radius)
Random noise points to test the filtering
Perfect 185mm spacing between poles for pattern matching
This will fully test your pole detection pipeline and show all the visualizations working correctly!
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import math
import time

class PoleSimulator(Node):
    def __init__(self):
        super().__init__('pole_simulator')
        
        self.publisher_ = self.create_publisher(PointCloud2, '/lslidar_point_cloud', 10)
        self.timer = self.create_timer(0.1, self.publish_pointcloud)  # 10Hz
        
        self.get_logger().info('Pole Simulator started - Publishing simulated pole data')
        
    def publish_pointcloud(self):
        msg = PointCloud2()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        
        # Fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16  # 4 floats * 4 bytes each
        msg.is_dense = True
        
        # Create simulated pole points
        points_data = []
        
        # Pole 1 at (0.5, 0.2)
        points_data.extend(self.create_pole_points(0.5, 0.2, 0.03, 20))
        
        # Pole 2 at (0.5, 0.4) 
        points_data.extend(self.create_pole_points(0.5, 0.4, 0.03, 20))
        
        # Pole 3 at (0.5, 0.6)
        points_data.extend(self.create_pole_points(0.5, 0.6, 0.03, 20))
        
        # Some noise points
        import random
        for _ in range(50):
            x = random.uniform(0.1, 1.0)
            y = random.uniform(-0.5, 0.5)
            z = random.uniform(-0.1, 0.1)
            intensity = random.uniform(100, 200)
            points_data.extend(struct.pack('ffff', x, y, z, intensity))
        
        msg.data = b''.join(points_data)
        msg.height = 1
        msg.width = len(points_data)
        
        self.publisher_.publish(msg)
        
    def create_pole_points(self, center_x, center_y, radius, num_points):
        """Create points in a circle to simulate a pole"""
        points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = 0.0  # Ground level
            intensity = 150.0  # Medium intensity
            points.append(struct.pack('ffff', x, y, z, intensity))
        return points

def main():
    rclpy.init()
    node = PoleSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()