# Pole Detection System - Quick Start Guide

## Quick Usage Instructions

### Start System Immediately

```bash
# Build workspace
colcon build --packages-select pole_detection lslidar_driver lslidar_msgs

# Source workspace
source install/setup.bash

# Launch
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true
```

**Note**: This automatically starts the N10-P LiDAR driver with default configuration (`/dev/ttyACM0`).

### LiDAR Driver Configuration

For detailed LiDAR driver setup, troubleshooting, and advanced configuration, see:

- **[LIDAR_DRIVER_GUIDE.md](LIDAR_DRIVER_GUIDE.md)** - Complete driver operations guide

**Quick Configuration Options**:

```bash
# Use different serial port
ros2 launch pole_detection pole_detection.launch.py serial_port:=/dev/ttyUSB0

# Use different LiDAR model
ros2 launch pole_detection pole_detection.launch.py lidar_model:=n10    # For N10
ros2 launch pole_detection pole_detection.launch.py lidar_model:=m10p  # For M10-P
ros2 launch pole_detection pole_detection.launch.py lidar_model:=m10   # For M10

# Production mode (no RViz)
ros2 launch pole_detection pole_detection.launch.py start_rviz:=false
```

**Note**: The LiDAR driver now starts automatically without manual lifecycle management. If you encounter "Node not found" errors, ensure you've rebuilt after recent changes:

```bash
colcon build --packages-select lslidar_driver
source install/setup.bash
```

### RViz Visualization

**Launch with RViz** (recommended - handles X11/Wayland issues automatically):

```bash
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true
```

**Manual RViz Launch** (if launching separately):

```bash
# Fix for Wayland/Snap conflicts
export QT_QPA_PLATFORM=xcb
rviz2 -d src/pole_detection/rviz/debug.rviz
```

**If RViz crashes with symbol lookup error**:

```bash
# This is a Snap package issue. Solutions:
# 1. Use the launch file method above (has fix built-in)
# 2. Or set environment variable before running rviz2:
export QT_QPA_PLATFORM=xcb

# 3. Or install non-snap version:
sudo apt install ros-humble-rviz2
```

### Using Action Server - CORRECTED COMMANDS

```bash
# Send action goal to start tracking (CORRECT COMMAND)
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback

# Monitor action feedback in real-time (CORRECT COMMAND)
ros2 topic echo /track_poles/_action/feedback

# Check action server status
ros2 action list

# Get action information
ros2 action info /track_poles

# Cancel action
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: false}"
```

### Key Topics for Monitoring

```bash
# Monitor detected poles
ros2 topic echo /detected_poles

# Check detection frequency (should be ~10Hz)
ros2 topic hz /detected_poles

# Monitor raw LiDAR data
ros2 topic echo /lslidar_point_cloud

# Monitor action feedback
ros2 topic echo /track_poles/_action/feedback
```

### Real-Time Parameter Tuning

**Manual Parameter Tuning:**

```bash
# List all parameters
ros2 param list | grep pole_detection

# Get current value
ros2 param get /pole_detection cluster_tolerance

# Set new value (no restart needed)
ros2 param set /pole_detection cluster_tolerance 0.08
ros2 param set /pole_detection ema_alpha 0.15
ros2 param set /pole_detection max_jump_distance 0.5
```

**For detailed parameter tuning guide, see:**

- [PARAMETER_TUNING_GUIDE.md](file:///home/rc3/Desktop/n10p_lidar/docs/PARAMETER_TUNING_GUIDE.md) - Complete vibration fix guide

---

## Vibration Fix Guide

### Problem: Excessive Dot Vibration

**Symptoms:**

- Tracking dots vibrate excessively even when stationary
- Jittery movement in RViz
- Unstable tracking

**Root Cause:**

- **NOT** algorithm choice (both EMA and Kalman work well)
- **Parameter mismatch** between clustering and tracking stages
- Tight clustering + loose association = unstable tracking

**Quick Fix:**

```bash
# 1. Launch system
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true

# 2. Monitor improvement
ros2 topic echo /debug/tracks
```

**Expected Results:**

- ✅ Stable tracking dots (minimal vibration)
- ✅ Smooth position updates
- ✅ No jumping between poles
- ✅ High tracking confidence

**For detailed troubleshooting, see:**

- [PARAMETER_TUNING_GUIDE.md](file:///home/rc3/Desktop/n10p_lidar/docs/PARAMETER_TUNING_GUIDE.md)

---

## Comprehensive Testing Guide

This section provides step-by-step testing procedures to verify all system components are working correctly: Detection, Visualization, ROS Communication, and Action Server.

### Phase 1: System Startup Verification

#### Step 1.1: Verify ROS2 Environment

```bash
# Check ROS2 is sourced
echo $ROS_DISTRO
# Expected: humble

# Check workspace is sourced
echo $AMENT_PREFIX_PATH
# Should contain your workspace path
```

#### Step 1.2: Build and Source

```bash
# Clean build (recommended after code changes)
rm -rf build install log
colcon build --packages-select pole_detection

# Source workspace
source install/setup.bash
```

### Phase 2: Detection Testing

#### Step 2.1: Verify LiDAR Data Flow

```bash
# Check if LiDAR point cloud is being published
ros2 topic info /lslidar_point_cloud
# Expected: Publisher count > 0, Type: sensor_msgs/msg/PointCloud2

# Monitor point cloud data (first 100 points)
ros2 topic echo /lslidar_point_cloud --field data --length 1 | head -100

# Check point cloud frequency
ros2 topic hz /lslidar_point_cloud
# Expected: ~10-20Hz depending on LiDAR model
```

#### Step 2.2: Test Pole Detection Pipeline

```bash
# Monitor detected objects
ros2 topic echo /detected_objects
# Should show DetectedObjects messages with object arrays

# Monitor detected poles specifically
ros2 topic echo /detected_poles
# Should show pole-specific detection data

# Check detection frequency
ros2 topic hz /detected_poles
# Expected: ~10Hz (matches LiDAR frequency)
```

### Phase 3: Visualization Testing

#### Step 3.1: Verify Debug Topics

```bash
# Check all debug visualization topics are active
ros2 topic list | grep debug
# Expected: /debug/clusters, /debug/accepted, /debug/rejected, /debug/pattern_matches, /debug/tracks

# Monitor cluster visualization
ros2 topic echo /debug/clusters --no-arr | head -5

# Monitor accepted/rejected candidates
ros2 topic echo /debug/accepted --no-arr | head -5
ros2 topic echo /debug/rejected --no-arr | head -5
```

#### Step 3.2: RViz Visualization Verification

```bash
# Launch RViz with debug configuration
ros2 run rviz2 rviz2 -d src/pole_detection/rviz/debug.rviz
```

**Expected RViz Visualizations:**
- ✅ **White Points**: LiDAR raw data (PointCloud2)
- ✅ **Orange Spheres**: Raw clusters (0.1m diameter)
- ✅ **Green Spheres**: Accepted candidates (0.08m diameter)
- ✅ **Red Spheres**: Rejected candidates (0.05m diameter)
- ✅ **Blue Lines**: Pattern matches between poles
- ✅ **Yellow/Green Spheres**: Tracked poles with ID labels

#### Step 3.3: TF Frame Verification

```bash
# Check TF frames are properly published
ros2 run tf2_tools view_frames
# Should show base_link → laser_link transform

# Monitor TF tree
ros2 run tf2_ros tf2_monitor
# Should show stable frame relationships
```

### Phase 4: ROS Communication Testing

#### Step 4.1: Node and Topic Verification

```bash
# List all active nodes
ros2 node list
# Expected: /pole_detection, /track_poles_action_server, /static_tf_pub, /rviz2

# List all active topics
ros2 topic list
# Should include all detection, debug, and action topics

# Check topic types
ros2 topic type /detected_poles
# Expected: pole_detection/msg/DetectedObjects

ros2 topic type /track_poles/_action/feedback
# Expected: pole_detection/action/TrackPoles_FeedbackMessage
```

#### Step 4.2: Parameter Verification

```bash
# Check all pole detection parameters
ros2 param list /pole_detection
# Should show cluster, validation, tracking, and debug parameters

# Verify critical parameters
ros2 param get /pole_detection publish_debug_clusters
# Expected: true

ros2 param get /pole_detection publish_debug_validation
# Expected: true

ros2 param get /pole_detection publish_debug_tracks
# Expected: true
```

### Phase 5: Action Server Testing

#### Step 5.1: Action Server Verification

```bash
# Check if action server is available
ros2 action list
# Should include /track_poles

# Get action server information
ros2 action info /track_poles
# Should show status and connected clients

# Check action interface
ros2 interface show pole_detection/action/TrackPoles
# Should show goal, result, and feedback definitions
```

#### Step 5.2: Action Server Functional Testing

```bash
# Send start tracking command
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback

# Monitor action feedback in real-time
ros2 topic echo /track_poles/_action/feedback
# Should show tracking status, pole counts, and confidence levels

# Send stop tracking command
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: false}"
```

#### Step 5.3: Action Server Integration Testing

```bash
# Test action with behavior tree integration
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true, timeout_seconds: 30}" --feedback

# Monitor action result
ros2 topic echo /track_poles/_action/result
# Should show final tracking results and statistics
```

### Phase 6: Performance Testing

#### Step 6.1: System Performance Monitoring

```bash
# Monitor system resource usage
ros2 run system_monitor system_monitor

# Check node performance
ros2 node info /pole_detection
# Should show publishers, subscribers, and services

# Monitor message latency
ros2 topic delay /detected_poles
# Should be < 100ms for real-time operation
```

#### Step 6.2: Detection Performance Testing

```bash
# Monitor detection statistics over time
ros2 topic hz /detected_poles --window 100
# Should maintain stable frequency

# Check detection latency
ros2 topic delay /lslidar_point_cloud /detected_poles
# Should be < 50ms for real-time detection
```

### Phase 7: Troubleshooting Common Issues

#### Issue: No LiDAR Data
```bash
# Check if LiDAR device is detected
ls /dev/ttyACM* /dev/ttyUSB*

# Check LiDAR driver status
ros2 node info /lslidar_driver_node

# Verify LiDAR parameters
ros2 param list /lslidar_driver_node
```

#### Issue: No Visualizations in RViz
```bash
# Check debug topics are publishing
ros2 topic info /debug/clusters

# Verify frame transforms
ros2 run tf2_ros tf2_echo base_link laser_link

# Check RViz configuration
cat src/pole_detection/rviz/debug.rviz | grep Frame
```

#### Issue: Action Server Not Responding
```bash
# Check action server status
ros2 node info /track_poles_action_server

# Verify action interface
ros2 interface show pole_detection/action/TrackPoles

# Test with simple goal
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}"
```

## Quick Testing Commands Summary

For rapid verification, use these essential commands:

```bash
# System health check
ros2 node list && ros2 topic list | grep -E "(debug|detected|track)"

# Detection verification
ros2 topic hz /detected_poles && ros2 topic echo /detected_poles --no-arr | head -3

# Visualization check
ros2 topic list | grep debug && ros2 topic info /debug/clusters

# Action server test
ros2 action list && ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback
```

This comprehensive testing guide ensures all system components are functioning correctly before deployment.

#### Step 1.3: Launch System

```bash
# Launch in debug mode with RViz
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true
```

**Expected Output:**

```
[INFO] [lslidar_driver_node]: LiDAR driver initialized
[INFO] [pole_detection]: Pole Detection Node initialized
[INFO] [pole_detection]: Pipeline: Clusterer → Validator → Tracker → PatternMatcher
[INFO] [pole_detection]: BT-Ready Action Server Started: /track_poles
[INFO] [pole_detection]: Clusterer debug publishing ENABLED
[INFO] [pole_detection]: Validator debug publishing ENABLED
[INFO] [pole_detection]: Tracker debug publishing ENABLED
[INFO] [pole_detection]: Pattern matcher debug publishing ENABLED
[INFO] [pole_detection]: Pipeline debug publishing ENABLED
```

### Phase 2: Hardware Verification

#### Step 2.1: Check LiDAR Connection

```bash
# Check LiDAR device permissions
ls -l /dev/ttyACM0
# Expected: crw-rw-rw- (read/write permissions)

# Fix permissions if needed
sudo chmod 666 /dev/ttyACM0

# Check LiDAR node is running
ros2 node list | grep lslidar
# Expected: /lslidar_driver_node
```

#### Step 2.2: Verify LiDAR Data

```bash
# Check if LiDAR is publishing
ros2 topic hz /lslidar_point_cloud
# Expected: ~10Hz (average rate: 10.000)

# Check point cloud data
ros2 topic echo /lslidar_point_cloud --once
# Should see PointCloud2 message with header and data
```

**If no LiDAR data:**

- Check LiDAR power: `lsusb | grep LiDAR`
- Check serial port: `dmesg | grep ttyACM`
- Check LiDAR node: `ros2 node list`
- Restart LiDAR if needed

### Phase 3: Topic Verification

#### Step 3.1: Verify All Debug Topics

```bash
# List all debug topics
ros2 topic list | grep debug
```

**Expected Output:**

```
/debug/clusters_raw
/debug/validated_poles
/debug/rejected_poles
/debug/tracks
/debug/pattern_matches
/debug/pipeline
```

#### Step 3.2: Check Each Debug Topic

```bash
# Check raw clusters
ros2 topic hz /debug/clusters_raw
# Expected: ~10Hz

# Check validated poles
ros2 topic hz /debug/validated_poles
# Expected: ~10Hz

# Check tracked poles
ros2 topic hz /debug/tracks
# Expected: ~10Hz

# Check pipeline status
ros2 topic hz /debug/pipeline
# Expected: ~10Hz
```

#### Step 3.3: Verify Output Topics

```bash
# Check detected poles output
ros2 topic hz /detected_poles
# Expected: ~10Hz

# View detected poles data
ros2 topic echo /detected_poles --once
# Should see DetectedObjects message with pole positions
```

### Phase 4: Action Server Verification

#### Step 4.1: Check Action Server

```bash
# List all actions
ros2 action list
```

**Expected Output:**

```
/track_poles
```

#### Step 4.2: Get Action Information

```bash
# Get detailed action info
ros2 action info /track_poles
```

**Expected Output:**

```
Action: /track_poles
Action Type: pole_detection/action/TrackPoles
Action Definition:
  Goal:
    bool start_tracking
  Result:
    bool success
  Feedback:
    int32 detected_poles_count
    geometry_msgs/Point[] pole_positions
    float32[] pole_distances_x
    float32[] pole_distances_y
    float32[] pole_confidences
```

#### Step 4.3: Test Action Server

```bash
# Send action goal (CORRECT COMMAND)
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback
```

**Expected Output:**

```
Waiting for an action server to become available...
Sending goal...
Goal accepted with ID: 1
Feedback:
  detected_poles_count: 6
  pole_positions:
    - {x: 0.45, y: 0.12, z: 0.0}
    - {x: 0.63, y: 0.12, z: 0.0}
    - {x: 0.81, y: 0.12, z: 0.0}
  pole_distances_x: [0.45, 0.63, 0.81]
  pole_distances_y: [0.12, 0.12, 0.12]
  pole_confidences: [0.95, 0.96, 0.97]
...
Goal finished with status: SUCCEEDED
```

#### Step 4.4: Monitor Action Feedback

```bash
# Monitor action feedback in real-time (CORRECT COMMAND)
ros2 topic echo /track_poles/_action/feedback
```

**Expected Continuous Output:**

```
---
feedback:
  detected_poles_count: 6
  pole_positions:
    - {x: 0.45, y: 0.12, z: 0.0}
    - {x: 0.63, y: 0.12, z: 0.0}
    - {x: 0.81, y: 0.12, z: 0.0}
  pole_distances_x: [0.45, 0.63, 0.81]
  pole_distances_y: [0.12, 0.12, 0.12]
  pole_confidences: [0.95, 0.96, 0.97]
---
feedback:
  detected_poles_count: 6
  pole_positions:
    - {x: 0.45, y: 0.12, z: 0.0}
    - {x: 0.63, y: 0.12, z: 0.0}
    - {x: 0.81, y: 0.12, z: 0.0}
  pole_distances_x: [0.45, 0.63, 0.81]
  pole_distances_y: [0.12, 0.12, 0.12]
  pole_confidences: [0.95, 0.96, 0.97]
---
```

### Phase 5: RViz Visualization Verification

#### Step 5.1: Check RViz Displays

In RViz, verify these displays are enabled:

**Required Displays:**

1. ✅ **Grid** - Reference frame
2. ✅ **LiDAR Raw Data** - Point cloud display
3. ✅ **Raw Clusters** - Orange spheres
4. ✅ **Validated Poles** - Green spheres
5. ✅ **Rejected Poles** - Yellow spheres
6. ✅ **Tracked Poles** - Blue/green spheres
7. ✅ **Pattern Matches** - Lines between poles
8. ✅ **Pipeline Status** - Text statistics
9. ✅ **TF Frames** - Coordinate frames

#### Step 5.2: Verify RViz Fixed Frame

```bash
# In RViz, check "Fixed Frame" dropdown
# Should be set to: laser_link
```

#### Step 5.3: Adjust RViz View

```bash
# In RViz, adjust camera to see poles:
# 1. Click "2D Goal Pose" tool
# 2. Click in scene to set view center
# 3. Use mouse to zoom and pan
# 4. Look for colored spheres and lines
```

#### Step 5.4: Manual RViz Setup (if needed)

If RViz doesn't show markers:

```bash
# 1. Launch RViz manually
rviz2

# 2. Add displays manually:
#    Click "Add" → "By topic" → "MarkerArray"
#    Select: /debug/clusters_raw → Enable
#    Select: /debug/validated_poles → Enable
#    Select: /debug/rejected_poles → Enable
#    Select: /debug/tracks → Enable
#    Select: /debug/pattern_matches → Enable
#    Select: /debug/pipeline → Enable

# 3. Add point cloud:
#    Click "Add" → "By topic" → "PointCloud2"
#    Select: /lslidar_point_cloud → Enable

# 4. Set fixed frame to: laser_link
```

### Phase 6: Console Log Monitoring

#### Step 6.1: Monitor System Logs

```bash
# Watch for detection logs in real-time
ros2 topic echo /rosout --filter "node_name=='pole_detection'"
```

**Key Log Messages to Monitor:**

```
✅ GOOD INDICATORS:
"Cluster 3: pts=10, bbox_area=0.0006m², convex_area=0.0005m², width=0.025m"
"✓ Pole 3 ACCEPTED: score=0.85 (ang=25°, pts=10, width=0.025m)"
"Track 2 UPDATED: pos=(0.45, 0.12), detections=5"
"✓ Pattern match: 100.0% (5/5 pairs)"
"Jump detected (0.520m) - resetting track 2"

❌ BAD INDICATORS:
"Cluster 5: REJECTED - only 2 points (hallucination)"
"Cluster 7: REJECTED - Wrong area (5000mm², expected 300-2500mm²)"
"Poles not colinear (tolerance: 0.02m)"
"✗ Pattern match: 0.0% (0/5 pairs)"
```

#### Step 6.2: Enable Debug Logging

```bash
# Launch with debug logging for detailed output
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true \
  --ros-args --log-level debug
```

---

## RViz Visualization Guide

### What You'll See in RViz

When you launch in debug mode, RViz will display **comprehensive enhanced visualization** of the entire detection pipeline with detailed labels, markers, and real-time statistics.

**For complete visualization guide, see:**

- [ENHANCED_VISUALIZATION_GUIDE.md](file:///home/rc3/Desktop/n10p_lidar/docs/ENHANCED_VISUALIZATION_GUIDE.md) - Complete visualization reference

### Enhanced Visualization Features

| Display Type              | Topic                      | Color              | Labels                             | Meaning                               |
| ------------------------- | -------------------------- | ------------------ | ---------------------------------- | ------------------------------------- |
| **Raw Clusters**    | `/debug/clusters_raw`    | 🟠 Orange          | "C0", "C1", "C2"...                | All candidate clusters with IDs       |
| **Validated Poles** | `/debug/validated_poles` | 🟢 Green           | "85%", "92%"...                    | Accepted poles with confidence scores |
| **Rejected Poles**  | `/debug/rejected_poles`  | 🔴 Red             | "Too few points", "Wrong width"... | Rejected clusters with reasons        |
| **Tracked Poles**   | `/debug/tracks`          | 🟢 Green/🟡 Yellow | "T0", "T1", "T2"...                | Tracked poles with IDs + arrows       |
| **Pattern Matches** | `/debug/pattern_matches` | 🟢 Green Lines     | "185mm", "187mm"...                | Pole spacing with distances           |
| **Pipeline Status** | `/debug/pipeline`        | 📊 Text + Bars     | Statistics + Progress bars         | Real-time pipeline monitoring         |

### Key Enhancements

✅ **Larger, more visible markers** (8-12cm diameter)
✅ **Detailed text labels** on all markers
✅ **Rejection reasons** on rejected poles
✅ **Confidence scores** on validated poles
✅ **Distance measurements** on pattern matches
✅ **Pipeline statistics** with progress bars
✅ **Real-time performance metrics**

### Understanding Pipeline Visualization

The `/debug/pipeline` topic shows complete data flow with enhanced statistics:

```
=== PIPELINE STATUS ===
Candidates: 12
Validated: 6
Rejected: 3
Tracked: 6
Pattern Matches: 5/5
Pattern Ratio: 100.00%
```

**Stage Progress Bars**: Visual representation of each pipeline stage count

### Quick Visual Diagnostics

1. **Orange Spheres Everywhere?**

   - **Problem**: Clustering too sensitive
   - **Solution**: Increase `cluster_tolerance` to 0.08m
   - **Command**: `ros2 param set /pole_detection cluster_tolerance 0.08`
2. **No Green Spheres?**

   - **Problem**: Validation too strict
   - **Solution**: Check red labels for rejection reasons, adjust thresholds
   - **Command**: `ros2 param set /pole_detection min_point_count 3`
3. **Yellow Spheres (Tentative Tracks)?**

   - **Problem**: Tracking not confirmed yet
   - **Solution**: Wait for 3+ detections or adjust `confirmation_threshold`
   - **Command**: `ros2 param set /pole_detection confirmation_threshold 2`
4. **No Green Lines?**

   - **Problem**: Spacing incorrect (not 185mm ±10mm)
   - **Solution**: Check pole placement or adjust `distance_tolerance`
   - **Command**: `ros2 param set /pole_detection distance_tolerance 0.01`
5. **Many Red Spheres?**

   - **Problem**: Validation rejecting too many candidates
   - **Solution**: Read rejection labels, adjust parameters accordingly
   - **Command**: Check red labels for specific reasons

**For detailed visualization guide and troubleshooting, see:**

- [ENHANCED_VISUALIZATION_GUIDE.md](file:///home/rc3/Desktop/n10p_lidar/docs/ENHANCED_VISUALIZATION_GUIDE.md)

---

## Common Issues and Solutions

### Issue 1: No Pole Detection

**Symptoms**: No green spheres, no detected poles

**Diagnosis Steps**:

```bash
# 1. Check LiDAR connection
ros2 topic hz /lslidar_point_cloud
# Expected: ~10Hz

# 2. Check node status
ros2 node list | grep pole_detection
# Expected: /pole_detection

# 3. Check for errors
ros2 topic echo /rosout --filter "node_name=='pole_detection'"

# 4. Check debug topics
ros2 topic list | grep debug
# Expected: 6 debug topics
```

**Solutions**:

- **Serial port permissions**: Add user to dialout group (required for /dev/ttyACM0 access):
  ```bash
  sudo usermod -a -G dialout $USER
  # Then logout and login again
  ```
- Check `/dev/ttyACM0` permissions: `sudo chmod 666 /dev/ttyACM0`
- Verify LiDAR is powered on
- Check range filters: `ros2 param get /pole_detection max_range`
- Adjust `max_range` to 1.0m if needed
- Restart system: `Ctrl+C` and relaunch

### Issue 2: False Positives (Walls Detected as Poles)

**Symptoms**: Too many green spheres, walls being detected

**Solutions**:

```bash
# Increase minimum width
ros2 param set /pole_detection min_radial_width 0.008

# Decrease maximum width
ros2 param set /pole_detection max_radial_width 0.035

# Enable stricter angular span
ros2 param set /pole_detection min_angular_span 20.0

# Monitor improvement
ros2 topic hz /debug/validated_poles
```

### Issue 3: Missed Detections

**Symptoms**: Poles visible but not detected

**Solutions**:

```bash
# Decrease minimum points per cluster
ros2 param set /pole_detection cluster_min_size 2

# Increase cluster tolerance
ros2 param set /pole_detection cluster_tolerance 0.05

# Relax validation thresholds
ros2 param set /pole_detection acceptance_threshold 0.50

# Monitor improvement
ros2 topic hz /debug/validated_poles
```

### Issue 4: Tracking Lag During Target Switches

**Symptoms**: Slow response when moving between poles

**Solutions**:

```bash
# Increase jump detection threshold
ros2 param set /pole_detection max_jump_distance 0.7

# Increase EMA alpha for faster response
ros2 param set /pole_detection ema_alpha 0.5

# Monitor improvement
ros2 topic echo /track_poles/_action/feedback
```

### Issue 5: Action Server Not Responding

**Symptoms**: Action goal not accepted, no feedback

**Diagnosis Steps**:

```bash
# 1. Check action server
ros2 action list
# Expected: /track_poles

# 2. View action info
ros2 action info /track_poles

# 3. Check node logs
ros2 topic echo /rosout --filter "node_name=='pole_detection'"
```

**Solutions**:

- Verify action server is running: `ros2 action list`
- Check action name: `/track_poles`
- Verify action type: `pole_detection/action/TrackPoles` (full package path required)
- Use correct command: `ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback`
- Restart node if needed

### Issue 6: RViz Not Showing Markers

**Symptoms**: RViz opens but no colored spheres or lines

**Diagnosis Steps**:

```bash
# 1. Check if debug topics are publishing
ros2 topic list | grep debug
# Expected: 6 debug topics

# 2. Check if topics have data
ros2 topic hz /debug/clusters_raw
# Expected: ~10Hz

# 3. View marker data
ros2 topic echo /debug/clusters_raw --once
# Should see MarkerArray message
```

**Solutions**:

- Verify RViz displays are enabled (check checkboxes)
- Check RViz fixed frame is set to `laser_link`
- Manually add displays if needed (see Phase 5.4)
- Restart RViz: Close and reopen
- Check RViz console for errors

### Issue 7: "The passed action type is invalid" Error

**Symptoms**: Action command fails with type error

**Diagnosis**:

```bash
# Check correct action type
ros2 interface show pole_detection/action/TrackPoles
```

**Solutions**:

- Use correct action type: `pole_detection/action/TrackPoles` (full package path required)
- Correct command: `ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback`
- Verify action is available: `ros2 action list`

---

## Performance Monitoring

### System Performance

```bash
# Monitor detection frequency
ros2 topic hz /detected_poles
# Expected: ~10Hz (matches LiDAR rate)

# Monitor bandwidth
ros2 topic bw /lslidar_point_cloud

# Monitor CPU usage
top | grep pole_detection
```

### Action Server Performance

```bash
# Monitor action feedback rate
ros2 topic echo /track_poles/_action/feedback
# Expected: 10Hz feedback

# Check feedback latency
# Should be <100ms from detection to feedback
```

### Pipeline Performance

Watch `/debug/pipeline` topic in RViz for real-time metrics:

- Processing Rate: 10Hz
- Latency: <10ms per frame
- Action Feedback: 10Hz

---

## Advanced Debugging

### Enable Detailed Logging

```bash
# Launch with debug logging
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true \
  --ros-args --log-level debug
```

### Monitor Individual Pipeline Stages

```bash
# Stage 1: Clustering
ros2 topic echo /debug/clusters_raw --once

# Stage 2: Validation
ros2 topic echo /debug/validated_poles --once

# Stage 3: Tracking
ros2 topic echo /debug/tracks --once

# Stage 4: Pattern Matching
ros2 topic echo /debug/pattern_matches --once

# Stage 5: Pipeline Status
ros2 topic echo /debug/pipeline --once
```

### Record and Replay

```bash
# Record all topics for analysis
ros2 bag record /lslidar_point_cloud /detected_poles /debug/* /track_poles/_action/feedback

# Replay for debugging
ros2 bag play recorded_bag.bag
```

### Parameter Exploration

```bash
# List all parameters
ros2 param list | grep pole_detection

# Get parameter value
ros2 param get /pole_detection cluster_tolerance

# Set parameter value
ros2 param set /pole_detection cluster_tolerance 0.05

# Reset to default
ros2 param dump /pole_detection > current_params.yaml
```

---

## System Overview

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Hardware Layer                        │
│  N10-P LiDAR → Serial (/dev/ttyACM0) @ 921600 baud       │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│               ROS2 Driver Layer                           │
│  lslidar_driver_node                                     │
│  - Publishes: /lslidar_point_cloud (PointCloud2)         │
│  - TF: laser_link coordinate frame                        │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│              Pole Detection Pipeline                       │
│                                                             │
│  Stage 1: Raw Point Cloud Conversion                       │
│    ↓                                                        │
│  Stage 2: Euclidean Clustering (4cm tolerance, 3 min pts)   │
│    ↓                                                        │
│  Stage 3: Multi-Feature Validation (distance-adaptive)      │
│    ↓                                                        │
│  Stage 4: World Frame Tracking (Smart EMA + Jump Detection)  │
│    ↓                                                        │
│  Stage 5: Strict Colinear Pattern Matching (185mm ±15mm)     │
│    ↓                                                        │
│  Output: /detected_poles                                   │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────┐
│              Action Server Layer                            │
│  track_poles_action_server                                │
│  - Action: /track_poles                                   │
│  - Multi-threaded execution (non-blocking)                │
│  - 10Hz feedback loop                                     │
│  - Thread-safe data access                                │
└─────────────────────────────────────────────────────────────┘
```

### Core Components

1. **LiDAR Driver** (`lslidar_driver`) - Interfaces with N10-P LiDAR hardware
2. **Pole Detection Pipeline** (`pole_detection`) - Main processing system with action server
3. **Action Server** (`track_poles`) - Handles pole tracking requests with real-time feedback
4. **TF2 Transform** - Manages coordinate frames

### Key Modules

- **Clusterer** - Extracts point clusters from raw LiDAR data
- **Validator** - Filters clusters using multi-feature scoring
- **Tracker** - Maintains pole tracks over time with jump detection
- **Pattern Matcher** - Identifies pole patterns (185mm spacing)
- **Action Server** - Provides BT-ready interface with 10Hz feedback

---

## Configuration Modes

### Production Mode (`production_params.yaml`)

- **Debug Publishing**: Disabled
- **Stricter Filters**: Higher validation thresholds
- **Performance Optimized**: Minimal computational overhead

### Debug Mode (`debug_params.yaml`)

- **Visualization**: All debug markers enabled
- **Relaxed Filters**: Lower thresholds for testing
- **Detailed Logging**: Comprehensive pipeline monitoring

---

## Technical Specifications

### LiDAR Specifications

- **Model**: N10-P
- **Scan Rate**: 10Hz
- **Points/Scan**: 3000
- **Angular Resolution**: 0.12°
- **Range**: 0.2m - 0.8m (optimized)

### Computational Requirements

- **CPU**: Moderate (single-core optimized)
- **Memory**: ~100MB typical usage
- **ROS2**: Foxy or newer required

### Detection Performance

- **Accuracy**: ±1cm position, ±2mm radius
- **Latency**: <100ms end-to-end
- **Range**: 0.2m - 0.8m optimal
- **False Positive Rate**: <5% (configurable)
- **Action Feedback Rate**: 10Hz

### Tracking Performance

- **Jump Detection**: <50ms response to target switches
- **EMA Smoothing**: Configurable (default 0.3)
- **Multi-Threading**: Non-blocking callbacks
- **Thread Safety**: Mutex-protected data access

---

## Maintenance Procedures

### Daily

- Monitor detection performance metrics
- Check for action server errors
- Verify feedback latency (<100ms)
- Check system logs

### Weekly

- Verify LiDAR calibration and mounting
- Check for software updates
- Test with known pole configurations
- Review tracking stability
- Backup configuration files

### Monthly

- Update parameters based on environment changes
- Backup configuration files
- Review and optimize performance
- Test action server integration with behavior trees
- Review system resource usage

---

## Related Documentation

- [README.md](file:///home/rc3/Desktop/n10p_lidar/docs/README.md) - Complete system documentation
- [COMPREHENSIVE_COMPARISON.md](file:///home/rc3/Desktop/n10p_lidar/docs/COMPREHENSIVE_COMPARISON.md) - Comparison with rc2026_head_finder

---

**Note**: This system is continuously improving. For latest updates and improvements, refer to Git history and changelog.