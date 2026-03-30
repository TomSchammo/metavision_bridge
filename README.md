# Metavision Bridge

ROS2 node that converts Prophesee `event_camera_msgs/EventPacket` to `dvs_msgs/EventArray` for use with ESVO.

## Convention

| Prophesee | ESVO | Description |
|-----------|------|-------------|
| master (cam_0) | left | Reference camera |
| slave (cam_1) | right | Secondary camera |
| T_slave_to_master | T_right_left | Extrinsic transformation |

**Perspective**: Left/right from behind the cameras (camera's viewpoint).

## Default Topic Mapping

| Input (Prophesee) | Output (ESVO) |
|-------------------|---------------|
| `/event_cam_0/events` | `/evk/left/events` |
| `/event_cam_1/events` | `/evk/right/events` |

## Build

```bash
colcon build --packages-select metavision_bridge
```

## Usage

### Stereo (default)

```bash
ros2 launch metavision_bridge metavision_bridge.launch.py
```

### Single camera

```bash
ros2 launch metavision_bridge metavision_bridge.launch.py stereo:=false
```

### Custom input topics

```bash
# If using single camera on default topic
ros2 launch metavision_bridge metavision_bridge.launch.py \
  stereo:=false \
  left_input:=/event_camera/events
```

### Flip left/right

If your master camera is physically on the right side:

```bash
ros2 launch metavision_bridge metavision_bridge.launch.py \
  left_input:=/event_cam_1/events \
  right_input:=/event_cam_0/events
```

### Custom output topics

To match existing ESVO topic names (e.g., for existing launch files):

```bash
ros2 launch metavision_bridge metavision_bridge.launch.py \
  left_output:=/davis/left/events \
  right_output:=/davis/right/events
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `stereo` | `true` | Launch both bridges (false for single camera) |
| `left_input` | `/event_cam_0/events` | Input topic for left (master) camera |
| `right_input` | `/event_cam_1/events` | Input topic for right (slave) camera |
| `left_output` | `/evk/left/events` | Output topic for left camera |
| `right_output` | `/evk/right/events` | Output topic for right camera |

## Calibration

Use the conversion script to convert Prophesee calibration to ESVO format:

```bash
python scripts/convert_prophesee_calib.py \
  --master-intrinsics /path/to/cam0_intrinsics.json \
  --slave-intrinsics /path/to/cam1_intrinsics.json \
  --extrinsics /path/to/extrinsics.json \
  --output-dir esvo_core/calib/evk4
```

This generates `left.yaml` and `right.yaml` with:
- Stereo rectification matrices (R1, R2)
- Projection matrices (P1, P2)
- Extrinsic transformation (T_right_left)

## Message Conversion

### Input: `event_camera_msgs/EventPacket`
- Encoded event data from Prophesee driver
- Decoded using `event_camera_codecs`

### Output: `dvs_msgs/EventArray`
```
std_msgs/Header header
  - stamp: first event's sensor timestamp
  - frame_id: copied from input
uint32 height
uint32 width
Event[] events
  - uint16 x, y
  - builtin_interfaces/Time ts (sensor timestamp)
  - bool polarity
```

## Notes

- Timestamps are sensor timestamps (not ROS time) for accurate stereo synchronization
- Hardware-synced cameras share the same time base
- The `header.stamp` is set to the first event's timestamp for ESVO's consistency checking
