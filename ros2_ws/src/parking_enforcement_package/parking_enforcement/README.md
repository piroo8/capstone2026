# `parking_enforcement` ROS 2 package

This package contains the parking-enforcement mission stack for the ROB498 capstone demo.
It stays separate from the older `flight3` package, but it preserves the same service interface:

- `/rob498_drone_8/comm/launch`
- `/rob498_drone_8/comm/test`
- `/rob498_drone_8/comm/land`
- `/rob498_drone_8/comm/abort`

The package is split into four main pieces:

- `stereo_depth_node.py`
  - subscribes to the RealSense fisheye stereo pair
  - rectifies stereo images and crops the invalid left strip
  - publishes `/stereo/depth`, `/stereo/depth/camera_info`, and `/stereo/disp_vis`
- `parking_mission_node.py`
  - first-pass waypoint mission
  - hardcoded parking waypoints from `config.py`
  - obstacle sidestep / back-up behavior from `/avoid_dir`
  - yaw-right + vertical scan at each spot
  - writes first-pass results to `~/.ros/parking_enforcement/plate_log.json`
- `parking_revisit_node.py`
  - same mission flow for the revisit pass
  - loads `plate_log.json`
  - rescans each parking spot
  - writes revisit results to `~/.ros/parking_enforcement/revisit_log.json`
- `local_costmap_node.py`
  - consumes `/stereo/depth` and `/stereo/depth/camera_info`
  - publishes `/local_grid`
  - makes decisions only from forward `left`, `center`, and `right` sectors
  - publishes `/avoid_dir` as `CLEAR`, `LEFT`, `RIGHT`, or `BLOCKED`
- `plate_reader_node.py`
  - wraps the TensorRT license-plate detector and recognizer
  - stays disabled outside scan states
  - confirms a plate only after 10 matching 7-character reads

## Depth pipeline

This package runs its own stereo depth node inside `parking_enforcement`.
That node publishes:

- `/stereo/depth`
- `/stereo/depth/camera_info`
- `/stereo/disp_vis`

## Where to edit things first

Edit [config.py](/home/jetson/rob498_2026/capstone2026/ros2_ws/src/parking_enforcement_package/parking_enforcement/parking_enforcement/config.py) before building.

Important fields there:

- `DRONE_NS`
- `PARKING_WAYPOINTS`
- `RGB_IMAGE_TOPIC`
- `LPD_ENGINE_PATH`
- `LPR_ENGINE_PATH`
- `REVISIT_OVERSTAY_THRESHOLD_SEC`

### Hardcoded waypoints

The demo now uses hardcoded waypoints instead of a runtime `PoseArray` topic.
Edit them here:

```python
PARKING_WAYPOINTS = [
    (-1.0, -1.0, 1.0),
    (1.0, -1.0, 2.0),
    (1.0, 1.0, 1.0),
    (-1.0, 1.0, 0.5),
]
```

## Build

```bash
cd /home/jetson/rob498_2026/capstone2026/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select perception parking_enforcement
source install/setup.bash
```

## Run order

### Terminal 1: MAVROS

Launch MAVROS the same way you already do for flight.

Example:

```bash
ros2 launch px4_autonomy_modules mavros.launch.py
```

### Terminal 2: External pose / vision bridge

Launch your external pose bridge separately if your flight stack needs it.

### Terminal 3: Camera publishers

Run the stereo cameras and your front RGB camera publisher.

### Terminal 4: First pass

```bash
cd /home/jetson/rob498_2026/capstone2026/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch parking_enforcement parking_first_pass.launch.py
```

### Start the mission

```bash
ros2 service call /rob498_drone_8/comm/launch std_srvs/srv/Trigger '{}'
ros2 service call /rob498_drone_8/comm/test std_srvs/srv/Trigger '{}'
```

### Revisit pass

After the first pass writes `~/.ros/parking_enforcement/plate_log.json`, run:

```bash
cd /home/jetson/rob498_2026/capstone2026/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch parking_enforcement parking_revisit.launch.py
```

Then call the same services:

```bash
ros2 service call /rob498_drone_8/comm/launch std_srvs/srv/Trigger '{}'
ros2 service call /rob498_drone_8/comm/test std_srvs/srv/Trigger '{}'
```

## Topics to check before flight

```bash
ros2 topic echo /avoid_dir
ros2 topic echo /plate_reader/current
ros2 topic echo /plate_reader/confirmed
ros2 topic echo /stereo/depth/camera_info
```

You should also see:

- `/local_grid`
- `/stereo/depth`
- `/stereo/disp_vis`
- `/plate_reader/debug_image`

## Stereo crop check

Your stereo disparity output has a left-side invalid strip caused by rectification / disparity overlap.
That strip should be cropped before the occupancy node uses the depth image.

To verify it:

1. View `/stereo/disp_vis`
2. Confirm the persistent dark-blue left strip is removed from the published image
3. Echo `/stereo/depth/camera_info`
4. Confirm the width is smaller than the raw rectified width and the principal point `cx` has shifted left accordingly

If the crop needs tuning, edit the crop constants in [config.py](/home/jetson/rob498_2026/capstone2026/ros2_ws/src/parking_enforcement_package/parking_enforcement/parking_enforcement/config.py).

## Mission behavior

1. `launch`
   - enters `OFFBOARD`
   - arms
   - climbs to `LAUNCH_ALT`
2. `test`
   - starts the waypoint mission using the hardcoded parking list
3. waypoint flight
   - if the center sector is clear, the mission continues normally
   - if the center sector is blocked, it chooses the freer side from left/right
   - if left, center, and right are all blocked, it backs up and re-evaluates
4. at each parking spot
   - holds position
   - yaws right
   - scans vertically
   - waits for a confirmed plate or timeout
5. end of mission
   - sends `AUTO.LAND`

## RViz debug topics

Recommended displays:

- `Map` -> `/local_grid`
- `Image` -> `/stereo/disp_vis`
- `Image` -> `/plate_reader/debug_image`
- `Pose` -> `/mavros/local_position/pose`

## Notes

- `land` is a normal `AUTO.LAND` request.
- `abort` also switches to `AUTO.LAND` for this demo.
- `home_pose` initializes from the first valid local pose sample.
- The occupancy map is local and reactive, not global SLAM.
- The planner uses only the forward `left`, `center`, and `right` sectors for decisions, even though the full 2D grid is still published for debugging.
