#!/usr/bin/env python3
# File: /home/rc3/Desktop/n10p_lidar/scripts/simple_visualizer.py

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class SimpleVisualizer(Node):
    def __init__(self):
        super().__init__('simple_visualizer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lslidar_point_cloud',
            self.pointcloud_callback,
            10)
        
        # Setup matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.scatter = self.ax.scatter([], [], s=1)
        
    def pointcloud_callback(self, msg):
        # Extract points from PointCloud2
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(points))
        
        if len(points) > 0:
            # Update plot
            self.scatter.set_offsets(points[:, :2])
            self.ax.set_title(f'LiDAR Points: {len(points)}')
            plt.pause(0.01)

def main():
    rclpy.init()
    node = SimpleVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
