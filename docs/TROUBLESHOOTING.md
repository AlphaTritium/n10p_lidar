# Troubleshooting Guide - Common Issues & Solutions

This document lists all problems encountered during development and deployment of the N10-P LiDAR pole detection system, along with their solutions.

---

## LiDAR Driver Issues

### Problem 1: Lifecycle Node Not Activating
**Error**: `Node not found` when checking `/lslidar_driver_node` lifecycle state  
**Solution**: Changed `LifecycleNode` to regular `Node` in all launch files (lsn10p_launch.py, lsn10_launch.py, lsm10p_uart_launch.py, lsm10_uart_launch.py) for automatic startup without manual lifecycle management

### Problem 2: Invalid Lifecycle Transition Arguments
**Error**: `Invalid arguments passed to constructor: transition` when trying to emit lifecycle events  
**Solution**: Removed complex lifecycle event emission code and simplified to regular Node initialization

### Problem 3: LiDAR Driver Not Publishing Data
**Error**: `/lslidar_point_cloud` topic shows no frequency or "does not appear to be published yet"  
**Solution**: Fixed lifecycle node activation issue (see Problem 1), rebuilt package with `colcon build --packages-select lslidar_driver`, and ensured proper serial port permissions

### Problem 4: Serial Port Permission Denied
**Error**: Cannot access `/dev/ttyACM0` device  
**Solution**: Added user to dialout group with `sudo usermod -aG dialout $USER` and logged out/in, or temporarily fixed with `sudo chmod 666 /dev/ttyACM0`

### Problem 5: Multiple Duplicate Nodes Running
**Error**: `ros2 node list` shows multiple instances of `/pole_detection`, `/static_tf_pub`, `/track_poles_action_server` with warning about nodes sharing exact names  
**Solution**: Killed duplicate processes with `pkill -f pole_detection && pkill -f track_poles && pkill -f static_transform_publisher`, always use Ctrl+C to stop before relaunching

---

## RViz Visualization Issues

### Problem 6: RViz Symbol Lookup Error
**Error**: `rviz2: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE`  
**Solution**: Set environment variable `export QT_QPA_PLATFORM=xcb` before launching RViz, or use launch file which has this fix built-in via `additional_env={'QT_QPA_PLATFORM': 'xcb'}`

### Problem 7: Wayland Display Conflict
**Error**: `Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome` followed by RViz crash  
**Solution**: Force X11 backend by setting `QT_QPA_PLATFORM=xcb` in environment or adding to ~/.bashrc, or install non-snap version with `sudo apt install ros-humble-rviz2`

---

## Build & Compilation Issues

### Problem 8: Package Not Found After Changes
**Error**: Launch fails after modifying driver code without rebuild  
**Solution**: Always rebuild affected packages with `colcon build --packages-select <package_name>` and source workspace with `source install/setup.bash`

### Problem 9: Syntax Errors in Launch Files
**Error**: Python syntax errors when editing launch files with complex lifecycle event handlers  
**Solution**: Simplified launch files by removing unnecessary event handlers, used `get_problems` tool to validate syntax before testing

---

## Documentation Issues

### Problem 10: References to Non-Existent Files
**Error**: Documentation referencing deleted or moved files causing confusion  
**Solution**: Removed references to non-existent LIDAR_DRIVER_GUIDE.md from README.md and Quick_Start.md, kept only updates to existing documentation files

### Problem 11: Unnecessary Documentation Files Created
**Error**: Created extra documentation files (LIDAR_DRIVER_GUIDE.md, MIGRATION_SUMMARY.md, PRE_DELETION_CHECKLIST.md) against user preferences  
**Solution**: Deleted all extra files immediately, updated only existing README.md and Quick_Start.md as per user workflow preferences

---

## Configuration Issues

### Problem 12: Wrong LiDAR Model Configuration
**Error**: wheeltec_lidar.launch.py defaulted to N10 model instead of required N10-P  
**Solution**: Deprecated wheeltec_lidar.launch.py entirely, integrated dynamic model selection into pole_detection.launch.py with `lidar_model` argument defaulting to 'n10p'

### Problem 13: Redundant Wrapper Launch File
**Error**: wheeltec_lidar.launch.py served no unique purpose, just wrapped other launch files  
**Solution**: Removed dependency on wrapper file, pole_detection.launch.py now directly includes correct driver launch files via IncludeLaunchDescription

---

## Runtime Issues

### Problem 14: Topic Not Publishing Despite Node Running
**Error**: Node appears in `ros2 node list` but topics show "does not appear to be published yet"  
**Solution**: Verified lifecycle state was unconfigured, manually activated with `ros2 lifecycle set /lslidar_driver_node configure` then `activate`, or switched to regular Node for auto-activation

### Problem 15: Incomplete Scan Data
**Error**: `/scan` topic not publishing despite point cloud working  
**Solution**: Ensured LiDAR completes full rotation (359° → 0° wrap-around), checked angle cropping parameters in YAML config, verified LiDAR motor is spinning

---

## System Integration Issues

### Problem 16: TF Tree Incomplete
**Error**: Missing transforms between coordinate frames causing visualization issues  
**Solution**: Verified static_transform_publisher is running with correct arguments: `['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link']`

### Problem 17: Action Server Not Receiving Data
**Error**: Track poles action server not providing feedback despite detection node running  
**Solution**: Verified topic remappings are correct, checked that `/detected_objects` topic is being published, confirmed action server subscription matches publisher

---

## Environment Issues

### Problem 18: Workspace Not Sourced
**Error**: `source: command not found` or packages not found  
**Solution**: Always run `source ~/Desktop/n10p_lidar/install/setup.bash` after building, add to ~/.bashrc for convenience

### Problem 19: USB Device Not Detected
**Error**: `/dev/ttyACM0` does not exist  
**Solution**: Check physical connection with `lsusb` and `dmesg | grep tty`, ensure CH340 driver loaded, verify VM USB passthrough if using virtual machine

---

## Performance Issues

### Problem 20: High CPU Usage
**Error**: System consuming excessive CPU resources  
**Solution**: Disabled debug topic publishing in params.yaml, reduced point cloud processing load by adjusting angle cropping range, disabled RViz when not needed

### Problem 21: Low Detection Frequency
**Error**: `/detected_objects` publishing at less than 10 Hz  
**Solution**: Optimized clustering parameters, reduced max_range to limit processing volume, verified LiDAR is actually publishing at 10 Hz first

---

## Migration & Refactoring Issues

### Problem 22: Broken References After File Moves
**Error**: Code references to old file paths after moving driver files  
**Solution**: Verified no code dependencies on n10p_driver package using grep, updated all launch file includes to point to correct lslidar_driver locations

### Problem 23: Conflicting Launch Configurations
**Error**: Multiple launch files with overlapping functionality causing confusion  
**Solution**: Consolidated into single entry point (pole_detection.launch.py) with dynamic model selection, deprecated redundant wrapper files

---

## Best Practices Learned

1. **Always rebuild after code changes**: `colcon build --packages-select <changed_packages>`
2. **Source workspace after build**: `source install/setup.bash`
3. **Kill old instances before relaunching**: Use Ctrl+C or pkill to avoid duplicates
4. **Use launch files for RViz**: They handle environment variables automatically
5. **Prefer regular Nodes over LifecycleNodes**: Unless lifecycle management is specifically needed
6. **Check serial permissions early**: Add user to dialout group during setup
7. **Validate with simple commands first**: `ros2 node list`, `ros2 topic list`, `ros2 topic hz`
8. **Keep documentation minimal**: Update existing files rather than creating new ones
9. **Test incrementally**: Verify each component works before integrating
10. **Use dynamic parameters**: Adjust with `ros2 param set` without restarting

---

## Quick Diagnostic Commands

```bash
# Check if nodes are running
ros2 node list

# Check topic publication rates
ros2 topic hz /lslidar_point_cloud
ros2 topic hz /detected_objects

# Verify lifecycle state (if using LifecycleNode)
ros2 lifecycle get /lslidar_driver_node

# Check serial device
ls -la /dev/ttyACM0
dmesg | grep tty

# View recent logs
ros2 log /lslidar_driver_node
ros2 log /pole_detection

# Kill duplicate processes
pkill -f pole_detection
pkill -f lslidar_driver
```

---

## Contact & Support

For issues not covered here:
- Check ROS2 Humble documentation: https://docs.ros.org/en/humble/
- Search ROS Answers: https://answers.ros.org/
- Review project-specific docs in `/docs/` directory
- Email LiDAR manufacturer: honghangli@lslidar.com

---

**Last Updated**: 2026-04-21  
**ROS2 Version**: Humble Hawksbill  
**LiDAR Model**: N10-P (primary), N10, M10, M10-P (supported)
