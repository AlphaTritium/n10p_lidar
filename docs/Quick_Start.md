# 柱体检测系统 - 快速入门指南

## 快速使用说明

### 立即启动系统

```bash
# 构建工作空间
colcon build --packages-select pole_detection lslidar_driver lslidar_msgs

# 源工作空间
source install/setup.bash

# 启动
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true
```

**注意**: 这会自动使用默认配置（`/dev/ttyACM0`）启动 N10-P 激光雷达驱动程序。

### 激光雷达驱动配置

有关详细的激光雷达驱动设置、故障排除和高级配置，请参阅相关文档。

**快速配置选项**:

```bash
# 使用不同的串口
ros2 launch pole_detection pole_detection.launch.py serial_port:=/dev/ttyUSB0

# 使用不同的激光雷达型号
ros2 launch pole_detection pole_detection.launch.py lidar_model:=n10    # 用于 N10
ros2 launch pole_detection pole_detection.launch.py lidar_model:=m10p  # 用于 M10-P
ros2 launch pole_detection pole_detection.launch.py lidar_model:=m10   # 用于 M10

# 生产模式（无 RViz）
ros2 launch pole_detection pole_detection.launch.py start_rviz:=false
```

**注意**: 激光雷达驱动程序现在自动启动，无需手动生命周期管理。如果遇到"Node not found"错误，请确保在最近的更改后重新构建：

```bash
colcon build --packages-select lslidar_driver
source install/setup.bash
```

### RViz 可视化

**使用 RViz 启动**（推荐 - 自动处理 X11/Wayland 问题）：

```bash
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true
```

**手动启动 RViz**（如果单独启动）：

```bash
# 修复 Wayland/Snap 冲突
export QT_QPA_PLATFORM=xcb
rviz2 -d src/pole_detection/rviz/debug.rviz
```

**如果 RViz 因符号查找错误崩溃**：

```bash
# 这是 Snap 包问题。解决方案：
# 1. 使用上面的启动文件方法（内置修复）
# 2. 或在运行 rviz2 之前设置环境变量：
export QT_QPA_PLATFORM=xcb

# 3. 或安装非 snap 版本：
sudo apt install ros-humble-rviz2
```

### 使用动作服务器 - 修正后的命令

```bash
# 发送动作目标以开始跟踪（正确命令）
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback

# 实时监控动作反馈（正确命令）
ros2 topic echo /track_poles/_action/feedback

# 检查动作服务器状态
ros2 action list

# 获取动作信息
ros2 action info /track_poles

# 取消动作
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: false}"
```

### 监控的关键话题

```bash
# 监控检测到的柱体
ros2 topic echo /detected_poles

# 检查检测频率（应为 ~10Hz）
ros2 topic hz /detected_poles

# 监控原始激光雷达数据
ros2 topic echo /lslidar_point_cloud

# 监控动作反馈
ros2 topic echo /track_poles/_action/feedback
```

### 实时参数调整

**手动参数调整**：

```bash
# 列出所有参数
ros2 param list | grep pole_detection

# 获取当前值
ros2 param get /pole_detection cluster_tolerance

# 设置新值（无需重启）
ros2 param set /pole_detection cluster_tolerance 0.08
ros2 param set /pole_detection ema_alpha 0.10
ros2 param set /pole_detection max_jump_distance 0.5
```

---

## 振动修复指南

### 问题：过多的点振动

**症状**：

- 即使静止时，跟踪点也会过度振动
- RViz 中抖动运动
- 跟踪不稳定

**根本原因**：

- **不是**算法选择（EMA 和卡尔曼滤波都工作良好）
- **参数不匹配**：聚类和跟踪阶段之间
- 紧密聚类 + 松散关联 = 不稳定跟踪

**快速修复**：

```bash
# 1. 启动系统
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true

# 2. 监控改进
ros2 topic echo /debug/tracks
```

**预期结果**：

- ✅ 稳定的跟踪点（最小振动）
- ✅ 平滑的位置更新
- ✅ 不会在柱体之间跳跃
- ✅ 高跟踪置信度

---

## 综合测试指南

本节提供逐步测试程序，以验证所有系统组件是否正常工作：检测、可视化、ROS 通信和动作服务器。

### 阶段 1：系统启动验证

#### 步骤 1.1：验证 ROS2 环境

```bash
# 检查 ROS2 是否已源
echo $ROS_DISTRO
# 预期：humble

# 检查工作空间是否已源
echo $AMENT_PREFIX_PATH
# 应包含您的工作空间路径
```

#### 步骤 1.2：构建和源

```bash
# 清理构建（代码更改后推荐）
rm -rf build install log
colcon build --packages-select pole_detection

# 源工作空间
source install/setup.bash
```

#### 步骤 1.3：启动系统

```bash
# 以调试模式启动，带 RViz
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true
```

**预期输出**：

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

### 阶段 2：硬件验证

#### 步骤 2.1：检查激光雷达连接

```bash
# 检查激光雷达设备权限
ls -l /dev/ttyACM0
# 预期：crw-rw-rw-（读写权限）

# 如需要，修复权限
sudo chmod 666 /dev/ttyACM0

# 检查激光雷达节点是否运行
ros2 node list | grep lslidar
# 预期：/lslidar_driver_node
```

#### 步骤 2.2：验证激光雷达数据

```bash
# 检查激光雷达是否正在发布
ros2 topic hz /lslidar_point_cloud
# 预期：~10Hz（平均速率：10.000）

# 检查点云数据
ros2 topic echo /lslidar_point_cloud --once
# 应看到带有头信息和数据的 PointCloud2 消息
```

**如果没有激光雷达数据**：

- 检查激光雷达电源：`lsusb | grep LiDAR`
- 检查串口：`dmesg | grep ttyACM`
- 检查激光雷达节点：`ros2 node list`
- 如需要，重启激光雷达

### 阶段 3：话题验证

#### 步骤 3.1：验证所有调试点话

```bash
# 列出所有调试点话
ros2 topic list | grep debug
```

**预期输出**：

```
/debug/clusters
/debug/validated_poles
/debug/rejected_poles
/debug/tracks
/debug/pattern_matches
/debug/pipeline
```

**注意**: 实际话题名称是 `/debug/clusters`，而不是 `/debug/clusters_raw`。

#### 步骤 3.2：检查每个调试点话

```bash
# 检查原始聚类
ros2 topic hz /debug/clusters
# 预期：~10Hz

# 检查已验证的柱体
ros2 topic hz /debug/validated_poles
# 预期：~10Hz

# 检查跟踪的柱体
ros2 topic hz /debug/tracks
# 预期：~10Hz

# 检查流水线状态
ros2 topic hz /debug/pipeline
# 预期：~10Hz
```

#### 步骤 3.3：验证输出话题

```bash
# 检查检测到的柱体输出
ros2 topic hz /detected_poles
# 预期：~10Hz

# 查看检测到的柱体数据
ros2 topic echo /detected_poles --once
# 应看到带有柱体位置的 DetectedObjects 消息
```

### 阶段 4：动作服务器验证

#### 步骤 4.1：检查动作服务器

```bash
# 列出所有动作
ros2 action list
```

**预期输出**：

```
/track_poles
```

#### 步骤 4.2：获取动作信息

```bash
# 获取详细动作信息
ros2 action info /track_poles
```

**预期输出**：

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

#### 步骤 4.3：测试动作服务器

```bash
# 发送动作目标（正确命令）
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback
```

**预期输出**：

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

#### 步骤 4.4：监控动作反馈

```bash
# 实时监控动作反馈（正确命令）
ros2 topic echo /track_poles/_action/feedback
```

**预期连续输出**：

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
```

### 阶段 5：RViz 可视化验证

#### 步骤 5.1：检查 RViz 显示

在 RViz 中，验证这些显示已启用：

**必需的显示**：

1. ✅ **网格** - 参考框架
2. ✅ **激光雷达原始数据** - 点云显示
3. ✅ **原始聚类** - 橙色球体
4. ✅ **已验证的柱体** - 绿色球体
5. ✅ **被拒绝的柱体** - 黄色球体
6. ✅ **跟踪的柱体** - 蓝色/绿色球体
7. ✅ **模式匹配** - 柱体之间的线
8. ✅ **流水线状态** - 文本文字统计
9. ✅ **TF 框架** - 坐标框架

#### 步骤 5.2：验证 RViz 固定框架

```bash
# 在 RViz 中，检查"Fixed Frame"下拉菜单
# 应设置为：laser_link
```

#### 步骤 5.3：调整 RViz 视图

```bash
# 在 RViz 中，调整相机以查看柱体：
# 1. 点击"2D Goal Pose"工具
# 2. 在场景中点击以设置视图中心
# 3. 使用鼠标缩放和平移
# 4. 查找彩色球体和线
```

#### 步骤 5.4：手动 RViz 设置（如需要）

如果 RViz 未显示标记：

```bash
# 1. 手动启动 RViz
rviz2

# 2. 手动添加显示：
#    点击"Add" → "By topic" → "MarkerArray"
#    选择：/debug/clusters → 启用
#    选择：/debug/validated_poles → 启用
#    选择：/debug/rejected_poles → 启用
#    选择：/debug/tracks → 启用
#    选择：/debug/pattern_matches → 启用
#    选择：/debug/pipeline → 启用

# 3. 添加点云：
#    点击"Add" → "By topic" → "PointCloud2"
#    选择：/lslidar_point_cloud → 启用

# 4. 将固定框架设置为：laser_link
```

### 阶段 6：控制台日志监控

#### 步骤 6.1：监控系统日志

```bash
# 实时监控检测日志
ros2 topic echo /rosout --filter "node_name=='pole_detection'"
```

**要监控的关键日志消息**：

```
✅ 良好指标：
"Cluster 3: pts=10, bbox_area=0.0006m², convex_area=0.0005m², width=0.025m"
"✓ Pole 3 ACCEPTED: score=0.85 (ang=25°, pts=10, width=0.025m)"
"Track 2 UPDATED: pos=(0.45, 0.12), detections=5"
"✓ Pattern match: 100.0% (5/5 pairs)"
"Jump detected (0.520m) - resetting track 2"

❌ 不良指标：
"Cluster 5: REJECTED - only 2 points (hallucination)"
"Cluster 7: REJECTED - Wrong area (5000mm², expected 300-2500mm²)"
"Poles not colinear (tolerance: 0.02m)"
"✗ Pattern match: 0.0% (0/5 pairs)"
```

#### 步骤 6.2：启用调试日志

```bash
# 使用调试日志启动以获取详细输出
ros2 launch pole_detection pole_detection.launch.py start_rviz:=true \
  --ros-args --log-level debug
```

---

## RViz 可视化指南

### 您将在 RViz 中看到的内容

当您在调试模式下启动时，RViz 将显示整个检测流水线的**全面增强可视化**，包括详细标签、标记和实时统计信息。

### 增强可视化功能

| 显示类型              | 话题                      | 颜色              | 标签                             | 含义                               |
| --------------------- | ------------------------- | ----------------- | -------------------------------- | ---------------------------------- |
| **原始聚类**          | `/debug/clusters`         | 🟠 橙色           | "C0", "C1", "C2"...             | 所有候选聚类及其 ID                 |
| **已验证的柱体**      | `/debug/validated_poles`  | 🟢 绿色           | "85%", "92%"...                 | 接受的柱体及置信度分数              |
| **被拒绝的柱体**      | `/debug/rejected_poles`   | 🔴 红色           | "点数太少", "宽度错误"...        | 被拒绝的聚类及原因                  |
| **跟踪的柱体**        | `/debug/tracks`           | 🟢 绿色/🟡 黄色   | "T0", "T1", "T2"...             | 跟踪的柱体及 ID + 箭头             |
| **模式匹配**          | `/debug/pattern_matches`  | 🟢 绿色线         | "185mm", "187mm"...             | 柱体间距及距离                     |
| **流水线状态**        | `/debug/pipeline`         | 📊 文本 + 条形图  | 统计 + 进度条                    | 实时流水线监控                     |

### 关键增强

✅ **更大、更可见的标记**（8-12cm 直径）
✅ **所有标记上的详细文本标签**
✅ **被拒绝柱体上的拒绝原因**
✅ **已验证柱体上的置信度分数**
✅ **模式匹配上的距离测量**
✅ **带进度条的流水线统计**
✅ **实时性能指标**

### 理解流水线可视化

`/debug/pipeline` 话题显示带有增强统计的完整数据流：

```
=== PIPELINE STATUS ===
Candidates: 12
Validated: 6
Rejected: 3
Tracked: 6
Pattern Matches: 5/5
Pattern Ratio: 100.00%
```

**阶段进度条**：每个流水线阶段计数的可视化表示

### 快速视觉诊断

1. **到处都是橙色球体？**
   - 这意味着聚类器检测到许多候选
   - 检查聚类容差是否太大
   - 尝试减小 `cluster_tolerance` 参数

2. **很多红色球体？**
   - 验证器正在拒绝许多候选
   - 检查拒绝原因标签
   - 可能需要调整验证参数

3. **没有绿色球体？**
   - 没有柱体通过验证
   - 检查激光雷达是否正确对准柱体
   - 验证检测范围是否在 0.2-0.7m 内

4. **跟踪点跳跃？**
   - 关联距离可能太大
   - 尝试减小 `association_distance` 参数
   - 检查 EMA alpha 是否太高

---

## 快速测试命令摘要

为了快速验证，使用这些基本命令：

```bash
# 系统健康检查
ros2 node list && ros2 topic list | grep -E "(debug|detected|track)"

# 检测验证
ros2 topic hz /detected_poles && ros2 topic echo /detected_poles --no-arr | head -3

# 可视化检查
ros2 topic list | grep debug && ros2 topic info /debug/clusters

# 动作服务器测试
ros2 action list && ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback
```

这个综合测试指南确保在部署之前所有系统组件都能正常工作。
