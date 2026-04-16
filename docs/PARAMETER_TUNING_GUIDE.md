# Parameter Tuning Guide - Vibration Fix

## 🎯 Overview

This guide explains how to fix excessive vibration in pole detection tracking through proper parameter tuning. The vibration issue is **not** caused by the choice of EMA vs Kalman filter - both work well when properly tuned. The real issue is **parameter mismatch** between clustering and tracking stages.

---

## 🔍 Understanding Vibration Causes

### What Causes Vibration?

Vibration in tracking dots occurs when:
1. **Over-segmentation**: Too many small clusters are created
2. **Loose Association**: Tracks jump between nearby clusters
3. **High EMA Alpha**: Too much weight on new measurements
4. **Parameter Mismatch**: Clustering and tracking parameters are not aligned

### Visual Symptoms in RViz

| Symptom | Root Cause | Solution |
|---------|------------|----------|
| **Jittery dots** (rapid small movements) | High EMA alpha | Lower `ema_alpha` to 0.1-0.15 |
| **Jumping between poles** | Loose association distance | Tighten `association_distance` to 0.05-0.06m |
| **Multiple dots for same pole** | Tight cluster tolerance | Increase `cluster_tolerance` to 0.08-0.10m |
| **Flickering dots** | Low cluster size threshold | Increase `cluster_min_size` to 4-5 |
| **Unstable tracking** | Parameter mismatch | Align clustering and tracking parameters |

---

## 📊 Parameter Analysis

### Current n10p_lidar Parameters (Problematic)

```yaml
# Debug Mode
cluster_tolerance: 0.04        # TOO TIGHT - creates too many clusters
cluster_min_size: 3            # TOO LOW - accepts noise
association_distance: 0.1       # TOO LOOSE - tracks jump between clusters
ema_alpha: 0.3                 # TOO HIGH - overreacts to noise

# Production Mode  
cluster_tolerance: 0.03        # EVEN TIGHTER - worse over-segmentation
cluster_min_size: 3            # TOO LOW - accepts noise
association_distance: 0.08      # TOO LOOSE - tracks jump between clusters
ema_alpha: 0.3                 # TOO HIGH - overreacts to noise
```

### rc2026_head_finder Parameters (Stable)

```yaml
cluster_tolerance: 0.10        # RELAXED - proper pole separation
min_points_per_cluster: 2      # REASONABLE - handles sparse returns
ema_alpha: 0.3                 # WORKS - because clustering is relaxed
max_jump_distance: 0.5         # GOOD - handles target switches
```

**Key Insight**: rc2026_head_finder uses a **relaxed clustering** approach, which creates fewer, more stable clusters. This makes tracking much easier and smoother.

---

## 🎯 Optimized Parameters

### Recommended Debug Parameters

```yaml
# Clusterer Parameters (RELAXED for stability)
cluster_tolerance: 0.08        # 8cm - proper pole separation (from rc2026)
cluster_min_size: 4            # 4 points - filter noise better
cluster_max_size: 40           # 40 points - allow full pole detection

# Tracker Parameters (TIGHTER for stability)
association_distance: 0.05     # 5cm - match relaxed clustering
max_invisible_frames: 20       # Keep tracks longer
confirmation_threshold: 2       # Confirm faster

# EMA Parameters (LOWER for smoothness)
ema_alpha: 0.15                # 15% new, 85% old - much smoother
max_jump_distance: 0.5         # 50cm - handle target switches
```

### Recommended Production Parameters

```yaml
# Clusterer Parameters (RELAXED for stability)
cluster_tolerance: 0.08        # 8cm - proper pole separation
cluster_min_size: 4            # 4 points - filter noise
cluster_max_size: 35           # 35 points - allow full pole detection

# Tracker Parameters (TIGHTER for stability)
association_distance: 0.05     # 5cm - match relaxed clustering
max_invisible_frames: 25       # Keep tracks longer
confirmation_threshold: 3       # Require more confirmation

# EMA Parameters (LOWER for smoothness)
ema_alpha: 0.10                # 10% new, 90% old - very smooth
max_jump_distance: 0.5         # 50cm - handle target switches
```

---

## 🔧 Step-by-Step Tuning Process

### Step 1: Start with Clustering Parameters

Clustering is the foundation - get this right first!

```bash
# Launch system in debug mode
ros2 launch pole_detection pole_detection_debug.launch.py serial_port:=/dev/ttyACM0

# Test different cluster tolerance values
ros2 param set /pole_detection cluster_tolerance 0.08
ros2 param set /pole_detection cluster_min_size 4
ros2 param set /pole_detection cluster_max_size 40

# Monitor cluster output
ros2 topic echo /debug/clusters_raw
```

**What to Watch For:**
- ✅ One cluster per pole (not multiple clusters for same pole)
- ✅ Clusters are stable (don't flicker on/off)
- ✅ No clusters for noise/walls

**If You See:**
- ❌ Multiple clusters for same pole → Increase `cluster_tolerance` to 0.10
- ❌ Clusters flickering on/off → Increase `cluster_min_size` to 5
- ❌ Walls being detected → Decrease `cluster_tolerance` to 0.06

### Step 2: Adjust Association Distance

Once clustering is stable, match association distance to cluster size.

```bash
# Set association distance to match cluster tolerance
ros2 param set /pole_detection association_distance 0.05

# Monitor tracking
ros2 topic echo /debug/tracks

# Watch for track jumping
ros2 topic echo /rosout --filter "node_name=='pole_detection'"
```

**What to Watch For:**
- ✅ Tracks stay with same pole (don't jump between poles)
- ✅ Smooth track updates (no sudden position changes)
- ✅ Stable track IDs (don't change frequently)

**If You See:**
- ❌ Tracks jumping between poles → Decrease `association_distance` to 0.04
- ❌ Tracks not following poles → Increase `association_distance` to 0.06
- ❌ Multiple tracks for same pole → Decrease `association_distance` to 0.04

### Step 3: Fine-tune EMA Alpha

Finally, adjust EMA alpha for smoothness.

```bash
# Start with conservative value
ros2 param set /pole_detection ema_alpha 0.15

# Test different values
ros2 param set /pole_detection ema_alpha 0.10  # Very smooth
ros2 param set /pole_detection ema_alpha 0.20  # More responsive

# Monitor tracking smoothness
ros2 topic echo /debug/tracks
```

**What to Watch For:**
- ✅ Smooth position updates (no jitter)
- ✅ Tracks respond to actual movement (not too slow)
- ✅ Stable tracking when stationary

**If You See:**
- ❌ Jittery dots → Lower `ema_alpha` to 0.10
- ❌ Slow response to movement → Increase `ema_alpha` to 0.20
- ❌ Tracks lagging behind poles → Increase `ema_alpha` to 0.20

### Step 4: Verify with Real Poles

Test with actual poles to confirm stability.

```bash
# Place poles at 185mm spacing
# Start action server
ros2 action send_goal /track_poles pole_detection/action/TrackPoles "{start_tracking: true}" --feedback

# Monitor feedback
ros2 topic echo /track_poles/_action/feedback

# Watch RViz for stable tracking
```

**Expected Results:**
- ✅ Stable pole count (e.g., 5 poles = 5 tracks)
- ✅ Minimal vibration when stationary
- ✅ Smooth response when moving
- ✅ High tracking confidence (>0.9)

---

## 🧪 Real-time Tuning Commands

### Quick Tuning Script

```bash
#!/bin/bash
# Quick tuning script for n10p_lidar

echo "=== n10p_lidar Parameter Tuning ==="
echo "1. Setting clustering parameters..."
ros2 param set /pole_detection cluster_tolerance 0.08
ros2 param set /pole_detection cluster_min_size 4
ros2 param set /pole_detection cluster_max_size 40

echo "2. Setting tracking parameters..."
ros2 param set /pole_detection association_distance 0.05
ros2 param set /pole_detection max_invisible_frames 20
ros2 param set /pole_detection confirmation_threshold 2

echo "3. Setting EMA parameters..."
ros2 param set /pole_detection ema_alpha 0.15
ros2 param set /pole_detection max_jump_distance 0.5

echo "4. Monitoring system..."
ros2 topic echo /debug/tracks
```

### Parameter Comparison Script

```bash
#!/bin/bash
# Compare current vs recommended parameters

echo "=== Parameter Comparison ==="
echo ""
echo "Current cluster_tolerance:"
ros2 param get /pole_detection cluster_tolerance
echo "Recommended: 0.08"
echo ""
echo "Current association_distance:"
ros2 param get /pole_detection association_distance
echo "Recommended: 0.05"
echo ""
echo "Current ema_alpha:"
ros2 param get /pole_detection ema_alpha
echo "Recommended: 0.15 (debug) / 0.10 (production)"
```

---

## 📈 Algorithm Insights

### Why EMA Works Well

**Exponential Moving Average (EMA)** is perfect for pole tracking because:

1. **Simple & Fast**: O(1) time complexity, no matrix operations
2. **Memory Efficient**: Only stores current position, no history
3. **Tunable**: Single parameter (`ema_alpha`) controls responsiveness
4. **Jump Detection**: Easy to detect and handle sudden position changes

**EMA Formula:**
```cpp
position_new = ema_alpha * measurement + (1.0 - ema_alpha) * position_old
```

**Effect of EMA Alpha:**
- `ema_alpha = 0.1`: Very smooth, slow response (90% old, 10% new)
- `ema_alpha = 0.3`: Balanced (70% old, 30% new) - **rc2026_head_finder default**
- `ema_alpha = 0.5`: Responsive, more jitter (50% old, 50% new)

### Why Kalman Filter is Overkill

**Kalman Filter** is powerful but unnecessary for pole tracking:

1. **Complex**: Requires state vector, covariance matrices, process noise, measurement noise
2. **Overkill**: Pole position is 2D, not high-dimensional
3. **Hard to Tune**: Multiple parameters (Q, R, F, H matrices)
4. **No Clear Benefit**: EMA performs equally well when properly tuned

**When Kalman Filter is Better:**
- High-dimensional state (position + velocity + acceleration)
- Complex motion models (non-linear dynamics)
- Variable measurement noise
- Need uncertainty estimates

**When EMA is Better:**
- Simple 2D position tracking
- Constant velocity assumption
- Stable measurement noise
- Need fast, simple implementation

### Parameter Interactions

**Clustering ↔ Tracking:**
- Tight clustering requires tight association
- Loose clustering allows loose association
- **Rule**: `association_distance ≈ cluster_tolerance / 2`

**EMA Alpha ↔ Cluster Stability:**
- Stable clusters allow lower EMA alpha
- Unstable clusters require higher EMA alpha
- **Rule**: Fix clustering first, then tune EMA

**Jump Detection ↔ Target Switching:**
- Large jump distance handles target switches
- Small jump distance prevents false jumps
- **Rule**: `max_jump_distance ≈ 3 * cluster_tolerance`

---

## 🎓 Common Mistakes to Avoid

### Mistake 1: Tuning EMA First

**Wrong:** Start with EMA alpha, expect it to fix everything
**Right:** Fix clustering first, then tune EMA

### Mistake 2: Tight Clustering + Loose Association

**Wrong:** `cluster_tolerance: 0.03` + `association_distance: 0.10`
**Right:** `cluster_tolerance: 0.08` + `association_distance: 0.05`

### Mistake 3: High EMA Alpha with Unstable Clustering

**Wrong:** `ema_alpha: 0.3` + `cluster_tolerance: 0.03`
**Right:** `ema_alpha: 0.15` + `cluster_tolerance: 0.08`

### Mistake 4: Ignoring Cluster Size Thresholds

**Wrong:** `cluster_min_size: 2` (accepts noise)
**Right:** `cluster_min_size: 4` (filters noise)

### Mistake 5: Not Testing with Real Poles

**Wrong:** Tune with simulated data only
**Right:** Test with actual 25mm poles at 185mm spacing

---

## 📝 Parameter Reference Table

| Parameter | Current (Debug) | Current (Prod) | Recommended (Debug) | Recommended (Prod) | Effect |
|-----------|-----------------|----------------|---------------------|-------------------|---------|
| **Clustering** |
| `cluster_tolerance` | 0.04m | 0.03m | **0.08m** | **0.08m** | Pole separation |
| `cluster_min_size` | 3 | 3 | **4** | **4** | Noise filtering |
| `cluster_max_size` | 30 | 25 | **40** | **35** | Full pole detection |
| **Tracking** |
| `association_distance` | 0.10m | 0.08m | **0.05m** | **0.05m** | Track stability |
| `max_invisible_frames` | 20 | 25 | **20** | **25** | Track persistence |
| `confirmation_threshold` | 2 | 3 | **2** | **3** | Track confirmation |
| **EMA** |
| `ema_alpha` | 0.3 | 0.3 | **0.15** | **0.10** | Smoothness |
| `max_jump_distance` | 0.5m | 0.5m | **0.5m** | **0.5m** | Target switching |

---

## 🚀 Quick Start Commands

### Apply Optimized Parameters (Debug Mode)

```bash
# Launch system
ros2 launch pole_detection pole_detection_debug.launch.py serial_port:=/dev/ttyACM0

# Apply optimized parameters
ros2 param set /pole_detection cluster_tolerance 0.08
ros2 param set /pole_detection cluster_min_size 4
ros2 param set /pole_detection cluster_max_size 40
ros2 param set /pole_detection association_distance 0.05
ros2 param set /pole_detection ema_alpha 0.15
ros2 param set /pole_detection max_jump_distance 0.5

# Monitor results
ros2 topic echo /debug/tracks
```

### Apply Optimized Parameters (Production Mode)

```bash
# Launch system
ros2 launch pole_detection pole_detection.launch.py serial_port:=/dev/ttyACM0

# Apply optimized parameters
ros2 param set /pole_detection cluster_tolerance 0.08
ros2 param set /pole_detection cluster_min_size 4
ros2 param set /pole_detection cluster_max_size 35
ros2 param set /pole_detection association_distance 0.05
ros2 param set /pole_detection ema_alpha 0.10
ros2 param set /pole_detection max_jump_distance 0.5

# Monitor results
ros2 topic echo /detected_poles
```

---

## ✅ Verification Checklist

After tuning, verify:

- [ ] Clusters are stable (one per pole, no flickering)
- [ ] Tracks don't jump between poles
- [ ] Tracking dots are smooth (minimal vibration)
- [ ] Tracking responds to actual movement
- [ ] Pole count is correct (e.g., 5 poles = 5 tracks)
- [ ] Tracking confidence is high (>0.9)
- [ ] Action feedback is stable
- [ ] RViz visualization is clean

---

## 📚 Additional Resources

- [Quick_Start.md](file:///home/rc2/FINN/pole/n10p_lidar/docs/Quick_Start.md) - System setup and usage
- [TECHNICAL_DEEP_DIVE.md](file:///home/rc2/FINN/pole/n10p_lidar/docs/TECHNICAL_DEEP_DIVE.md) - Algorithm details
- [COMPREHENSIVE_COMPARISON.md](file:///home/rc2/FINN/pole/n10p_lidar/docs/COMPREHENSIVE_COMPARISON.md) - Comparison with rc2026_head_finder

---

**Remember**: The key to stable tracking is **proper parameter tuning**, not algorithm choice. EMA works perfectly when parameters are correctly aligned! 🎯