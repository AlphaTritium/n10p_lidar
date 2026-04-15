# LiDAR Pole Detection System - Troubleshooting Guide

## Overview

This guide covers common issues and solutions for the ROS2-based pole detection system using N10-P LiDAR.

## Quick Status Check

### System Health Commands
```bash
# Check if LiDAR driver is running
ps aux | grep lslidar

# Check ROS2 topics
ros2 topic list | grep -E "(lslidar|detected)"

# Monitor LiDAR data flow
ros2 topic hz /lslidar_point_cloud
ros2 topic echo /detected_poles

# Check system performance
ros2 topic bw /lslidar_point_cloud
```

---

## Visual Debugging with RViz2

### Launching RViz2 for Visualization

#### Method 1: Pre-configured Debug Visualization (Recommended)
```bash
# Launch the pole detection system first
ros2 launch pole_detection pole_detection_debug.launch.py

# In a new terminal, launch RViz with debug configuration
rviz2 -d src/pole_detection/config/debug_visualization.rviz
```

#### Method 2: Manual RViz Setup
```bash
# Launch RViz2
rviz2

# Then manually add these displays:
# 1. Add → PointCloud2 → Topic: /lslidar_point_cloud
# 2. Add → MarkerArray → Topic: /debug/clusters_raw
# 3. Add → MarkerArray → Topic: /debug/tracks
# 4. Add → MarkerArray → Topic: /debug/pattern_matches
# 5. Add → Grid (for reference)
```

#### Method 3: Launch Everything Together
```bash
# Launch system with RViz automatically
ros2 launch pole_detection pole_detection_debug.launch.py &
sleep 5
rviz2 -d src/pole_detection/config/debug_visualization.rviz
```

### What to Look For in RViz

#### Point Cloud Visualization
- **Topic**: `/lslidar_point_cloud`
- **Appearance**: Colored points showing LiDAR returns
- **Expected**: Concentric rings of points around the sensor
- **Issues**: 
  - No points → LiDAR not connected
  - Sparse points → Range too far or obstacles blocking
  - Distorted rings → LiDAR calibration issues

#### Raw Clusters (Debug Mode)
- **Topic**: `/debug/clusters_raw`
- **Appearance**: Colored spheres around detected clusters
- **Colors**: 
  - Blue: Small clusters
  - Green: Medium clusters  
  - Red: Large clusters
- **Expected**: Spheres around pole-like objects

#### Tracked Poles
- **Topic**: `/debug/tracks`
- **Appearance**: Colored spheres with ID labels
- **Colors**:
  - Yellow: Tentative tracks (not confirmed)
  - Green: Confirmed tracks
- **Expected**: Stable spheres at pole positions

#### Pattern Matches
- **Topic**: `/debug/pattern_matches`
- **Appearance**: Lines connecting poles with 185mm spacing
- **Expected**: Straight lines connecting poles in sequence

### RViz Display Configuration Tips

#### Optimize Point Cloud Display
```yaml
# In PointCloud2 display settings:
Style: Points
Size (Pixels): 3
Alpha: 1.0
Color Transformer: Intensity
```

#### Configure Marker Display
```yaml
# In MarkerArray display settings:
Namespaces:
  - clusters
  - tracks  
  - pattern_matches
Queue Size: 10
```

#### Set Fixed Frame
- **Fixed Frame**: `laser_link` (for sensor view) or `base_link` (for robot view)
- **TF**: Ensure `base_link` → `laser_link` transform is published

### Debug Mode vs Production Mode

#### Debug Mode (Recommended for Visualization)
```bash
ros2 launch pole_detection pole_detection_debug.launch.py
```
**Features**:
- Publishes all debug topics
- Shows raw clusters, tracks, and pattern matches
- Higher verbosity logging
- Slower performance but better visualization

#### Production Mode
```bash
ros2 launch pole_detection pole_detection.launch.py
```
**Features**:
- Minimal debug publishing
- Optimized for performance
- Only publishes final detected poles
- Less visualization but faster operation

### Step-by-Step Visual Debugging

#### Step 1: Verify LiDAR Data
```bash
# Launch system
ros2 launch pole_detection pole_detection_debug.launch.py

# Launch RViz
rviz2 -d src/pole_detection/config/debug_visualization.rviz

# Check: Do you see point cloud data in RViz?
# If not, check LiDAR connection and permissions
```

#### Step 2: Check Clustering
```bash
# In RViz, enable the clusters display
# Look for colored spheres around potential poles
# Adjust cluster parameters if needed:
ros2 param set /pole_detection cluster_tolerance 0.04
ros2 param set /pole_detection cluster_min_size 3
```

#### Step 3: Validate Pole Detection
```bash
# Enable tracks display in RViz
# Look for green confirmed tracks at pole positions
# Check if poles are being correctly identified
```

#### Step 4: Verify Pattern Matching
```bash
# Enable pattern matches display
# Look for lines connecting poles with 185mm spacing
# Verify the pattern detection is working
```

### Common RViz Issues and Solutions

#### Issue: No Data in RViz
```bash
# Check if topics are publishing
ros2 topic list
ros2 topic echo /lslidar_point_cloud --no-arr | head -5

# Check TF frame
ros2 topic echo /tf

# Verify RViz configuration
# Fixed Frame should match your TF tree
```

#### Issue: Wrong Coordinate Frame
```bash
# Check available TF frames
ros2 run tf2_tools view_frames.py

# Ensure static transform is publishing
ros2 topic echo /tf_static

# In RViz: Fixed Frame → laser_link or base_link
```

#### Issue: Performance Lag
```bash
# Reduce point cloud size in RViz
# Set Decay Time to 0.1 seconds
# Reduce marker queue size
# Use production mode for better performance
```

### Advanced Visualization Tips

#### Record and Playback
```bash
# Record session for analysis
ros2 bag record -o debug_session /lslidar_point_cloud /detected_poles /debug/*

# Playback for detailed analysis
ros2 bag play debug_session
rviz2 -d src/pole_detection/config/debug_visualization.rviz
```

#### Custom RViz Configurations
```bash
# Save your current RViz configuration
# File → Save Config As → my_config.rviz

# Load custom configuration
rviz2 -d my_config.rviz
```

#### Multiple Viewports
- **View 1**: Top-down for pattern visualization
- **View 2**: First-person for LiDAR perspective
- **View 3**: Side view for height verification

---

### Expected Output When Working

- **LiDAR Driver**: Should show `[CRC] Expected: X, Got: X, Match: 1`
- **Point Cloud**: `/lslidar_point_cloud` topic publishing at ~10Hz
- **Detection**: `/detected_poles` topic publishing detected objects

---

## Common Issues & Solutions

### 1. LiDAR Device Not Detected

**Symptoms**:

- `[ERROR] [lslidar_driver_node]: Failed to open serial port`
- No `/dev/ttyACM0` or `/dev/ttyUSB0` device

**Solutions**:

#### A. Check Physical Connection

```bash
# List USB devices
lsusb

# Check for serial devices
ls -la /dev/tty* | grep -E "(ACM|USB)"

# Check kernel messages for device detection
dmesg | grep -i "usb\|tty\|acm" | tail -20
```

#### B. Fix Permissions (Most Common Issue)

```bash
# Check current permissions
ls -la /dev/ttyACM0

# Fix permissions temporarily
sudo chmod 666 /dev/ttyACM0

# Permanent fix - add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in after this command
```

#### C. VMware USB Passthrough (Virtual Machines)

1. **VM → Removable Devices** → Find LiDAR → Click "Connect"
2. **VM → Settings → USB Controller** → Enable "Show all USB input devices"
3. Try different USB compatibility modes (USB 2.0 vs 3.0)

#### D. Try Different Device Paths

```bash
# Common LiDAR device paths
ros2 launch pole_detection pole_detection.launch.py serial_port:=/dev/ttyUSB0
ros2 launch pole_detection pole_detection.launch.py serial_port:=/dev/ttyACM1
ros2 launch pole_detection pole_detection.launch.py serial_port:=/dev/ttyS0
```

### 2. LiDAR Connected But No Data

**Symptoms**:

- Device appears in `/dev/` but no point cloud data
- LiDAR driver starts but shows no activity

**Solutions**:

#### A. Verify LiDAR Power and Status

- Check LiDAR status LEDs (should be solid/active)
- Ensure proper power supply
- Try different USB cable (some are power-only)

#### B. Check Baud Rate Configuration

```bash
# Verify baud rate in launch file matches LiDAR specs
# N10-P typically uses 460800 baud
cat src/pole_detection/launch/pole_detection.launch.py | grep baud_rate
```

#### C. Test Raw LiDAR Communication

```bash
# Test if LiDAR sends raw data
sudo screen /dev/ttyACM0 460800
# Or use minicom
sudo apt install minicom
minicom -D /dev/ttyACM0 -b 460800
```

### 3. Pole Detection Not Working

**Symptoms**:

- Point cloud data flowing but no poles detected
- `/detected_poles` topic empty

**Solutions**:

#### A. Check Detection Parameters

```bash
# View current parameters
ros2 param list
ros2 param get /pole_detection cluster_tolerance
ros2 param get /pole_detection cluster_min_size

# Modify parameters for testing
ros2 param set /pole_detection cluster_tolerance 0.05
ros2 param set /pole_detection cluster_min_size 2
```

#### B. Enable Debug Mode

```bash
# Launch with debug parameters
ros2 launch pole_detection pole_detection_debug.launch.py

# Check debug topics
ros2 topic list | grep debug
ros2 topic echo /debug/clusters_raw
```

#### C. Verify Pole Characteristics

- Ensure poles are within 0.2m - 0.8m range
- Check pole diameter matches expected ~56mm
- Verify sufficient LiDAR points on poles (≥3 points)

### 4. System Performance Issues

**Symptoms**:

- High CPU usage
- Dropped frames or laggy detection
- Memory leaks

**Solutions**:

#### A. Optimize Parameters for Performance

```yaml
# In config/production_params.yaml
publish_debug_clusters: false
publish_debug_validation: false
publish_debug_tracks: false
publish_debug_pattern: false
cluster_max_size: 25  # Reduce from default
max_tracks: 8         # Reasonable limit
```

#### B. Monitor System Resources

```bash
# Monitor CPU and memory
htop

# Check ROS2 node performance
ros2 node info /pole_detection
ros2 node info /lslidar_driver_node
```

#### C. Reduce Computational Load

- Increase `cluster_tolerance` to 0.04-0.05m
- Set `cluster_min_size` to 3 (minimum viable)
- Reduce `max_range` to 0.7m if possible

---

## Debugging Workflow

### Step 1: Verify LiDAR Connection

```bash
# 1. Check device exists and permissions
ls -la /dev/ttyACM0

# 2. Test LiDAR driver independently
ros2 run lslidar_driver lslidar_driver_node

# 3. Check for CRC success messages
# Should see: [CRC] Expected: X, Got: X, Match: 1
```

### Step 2: Verify Data Flow

```bash
# 1. Check point cloud publishing
ros2 topic hz /lslidar_point_cloud

# 2. Visualize point cloud in RViz2
rviz2 -d src/pole_detection/config/debug_visualization.rviz

# 3. Check point cloud content
ros2 topic echo /lslidar_point_cloud --no-arr | head -20
```

### Step 3: Verify Detection Pipeline

```bash
# 1. Check detection topics
ros2 topic echo /detected_poles

# 2. Enable debug mode for detailed logging
ros2 launch pole_detection pole_detection_debug.launch.py

# 3. Monitor detection pipeline
ros2 topic echo /debug/clusters_raw
ros2 topic echo /debug/tracks
```

### Step 4: Performance Optimization

```bash
# 1. Monitor system resources
htop

# 2. Check topic bandwidth
ros2 topic bw /lslidar_point_cloud

# 3. Tune parameters based on environment
ros2 param set /pole_detection cluster_tolerance 0.04
ros2 param set /pole_detection max_range 0.7
```

---

## Success Indicators

### When System is Working Correctly

**LiDAR Driver Logs**:

```
[INFO] [lslidar_driver_node]: [DEBUG] polling: receive_data returned len=108
[INFO] [lslidar_driver_node]: [DEBUG] data_processing_2: len=108, PACKET_SIZE=108
[INFO] [lslidar_driver_node]: [CRC] Expected: 0xC9, Got: 0xC9, Match: 1
```

**Pole Detection Logs**:

```
[INFO] [pole_detection]: Pipeline: Clusterer → Validator → Tracker → PatternMatcher
[INFO] [pole_detection]: All modules initialized
[INFO] [pole_detection]: STRICT COLINEAR Pattern: XX.X% (X/X consecutive pairs match 185mm)
```

**Topic Activity**:

- `/lslidar_point_cloud`: Publishing at ~10Hz
- `/detected_poles`: Publishing detected objects
- `/detected_objects`: Publishing all valid clusters

---

## Emergency Recovery

### If System Completely Fails

1. **Restart Everything**:

```bash
# Kill all ROS2 processes
pkill -f ros2

# Restart from clean state
source install/setup.bash
ros2 launch pole_detection pole_detection.launch.py
```

2. **Reset Parameters to Defaults**:

```bash
# Relaunch with default parameters
ros2 launch pole_detection pole_detection.launch.py
```

3. **Check System Logs**:

```bash
# View system logs
journalctl -u ros2 --since "1 hour ago"

# Check ROS2 daemon status
ros2 daemon status
```

---

## Support Information

### Log Files Location

- **ROS2 Logs**: `~/.ros/log/`
- **System Logs**: `/var/log/syslog`
- **Application Logs**: Check terminal output

### Useful Debug Commands

```bash
# Get detailed node information
ros2 node info /pole_detection

# Check parameter values
ros2 param dump /pole_detection

# Monitor topic traffic
ros2 topic info /lslidar_point_cloud

# Record and replay data
ros2 bag record -o lidar_data /lslidar_point_cloud /detected_poles
```

### Contact for Support

- Check system logs for specific error messages
- Ensure all dependencies are installed
- Verify LiDAR hardware functionality
- Review parameter configurations

---

*Last Updated: 2026-04-09*
*Based on successful debugging session with N10-P LiDAR*