# originbot_traffic_light

ROS 2 node that detects traffic-light state from a camera image and publishes an
`originbot_msgs/TrafficLight` message on `/traffic_light/state`.

**Publish semantics:** a state message is emitted on *every* processed image
frame.  When detection fails or confidence is below threshold the message carries
`state = UNKNOWN` and `confidence = 0.0`, so downstream nodes and tools like
`ros2 topic echo` / `ros2 topic hz` never hang waiting for a first message.

## Detection modes

The node supports two independent detection modes, selected by the `mode` parameter.

### `roi` mode (default)

Fixed absolute-pixel regions-of-interest (ROIs) are evaluated on the full image.
Three zones (left / mid / right) are tested for green and red using HSV masks.
This is the original behaviour and is unchanged.

### `card` mode

Automatically detects a **printed traffic-light card** (white rounded rectangle
containing red arrows and one green arrow/circle) anywhere in the camera frame.
No manual ROI tuning is required.

Algorithm:
1. Convert frame to grayscale → Gaussian blur → Canny edge detection.
2. Find external contours, approximate each with `approxPolyDP`.
3. Keep 4-point polygons whose bounding-rect area exceeds `min_card_area` and
   whose aspect ratio is within `[card_aspect_ratio_min, card_aspect_ratio_max]`.
4. Select the largest qualifying candidate.
5. Apply a perspective transform to warp the card to a canonical
   `card_warp_width × card_warp_height` rectangle.
6. Split the warped card into three equal horizontal bands (left / middle / right)
   with an optional pixel margin (`band_margin_px`) on each side.
7. Compute the green-pixel ratio in each band using the same HSV thresholds as
   the ROI mode.
8. Determine the output state:
   - green mostly in left band → `LEFT`
   - green mostly in middle band → `STRAIGHT`
   - green mostly in right band → `RIGHT`
   - no band exceeds `min_green_ratio` → `STOP`
   - no valid card found → `UNKNOWN` (confidence 0.0)

## Quickstart

```bash
# Source your workspace
source /opt/ros/foxy/setup.bash
source ~/ws_originbot/install/setup.bash

# Run with default roi mode
ros2 launch originbot_traffic_light traffic_light_detector.launch.py

# Run in card mode (override a single parameter)
ros2 launch originbot_traffic_light traffic_light_detector.launch.py \
  --ros-args -p mode:=card

# Enable debug images in card mode
ros2 launch originbot_traffic_light traffic_light_detector.launch.py \
  --ros-args -p mode:=card -p debug:=true
```

To switch permanently, edit `config/params.yaml` and change `mode: roi` to
`mode: card`.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `image_topic` | string | `/image_raw` | Input image topic (override with `image_topic:=<topic>`) |
| `output_topic` | string | `/traffic_light/state` | Output state topic |
| `mode` | string | `roi` | Detection mode: `roi` or `card` |
| `left_x/y/w/h` | int | (see yaml) | Left ROI (roi mode) |
| `mid_x/y/w/h` | int | (see yaml) | Mid ROI (roi mode) |
| `right_x/y/w/h` | int | (see yaml) | Right ROI (roi mode) |
| `green_h/s/v_low/high` | int | (see yaml) | HSV green thresholds (both modes) |
| `red1_*/red2_*` | int | (see yaml) | HSV red thresholds (roi mode) |
| `min_green_ratio` | float | `0.01` | Min green pixel ratio to count as green |
| `min_red_ratio` | float | `0.01` | Min red pixel ratio to count as red (roi mode) |
| `card_warp_width` | int | `320` | Canonical warp width in pixels (card mode) |
| `card_warp_height` | int | `120` | Canonical warp height in pixels (card mode) |
| `min_card_area` | int | `5000` | Min contour area (px²) to consider as card |
| `card_aspect_ratio_min` | float | `1.5` | Min card width/height ratio |
| `card_aspect_ratio_max` | float | `6.0` | Max card width/height ratio |
| `band_margin_px` | int | `5` | Horizontal margin inside each band (card mode) |
| `debug` | bool | `false` | Publish annotated debug image (card mode) |
| `debug_topic` | string | `/traffic_light/debug` | Debug image topic |
| `publish_unknown` | bool | `true` | Publish `UNKNOWN` state when no confident detection; set `false` to revert to legacy publish-on-detection-only behaviour |

## Published topics

| Topic | Type | Description |
|---|---|---|
| `/traffic_light/state` | `originbot_msgs/TrafficLight` | Detected state + confidence (published every frame; `UNKNOWN` / `0.0` when no detection) |
| `/traffic_light/debug` | `sensor_msgs/Image` | Debug image (card mode, `debug:=true`) |

## Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | Input camera frame (default; OriginBot camera publishes here) |

## Manual verification

After starting the robot with camera enabled and launching the detector:

```bash
# Confirm messages arrive at a non-zero rate
ros2 topic hz /traffic_light/state

# Inspect messages – should show UNKNOWN when no card/sign is visible
ros2 topic echo /traffic_light/state --qos-reliability best_effort
```

Expected output when no traffic-light card is in frame:
```
header: ...
state: 0        # UNKNOWN
confidence: 0.0
```
