import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from originbot_msgs.msg import TrafficLight
from rclpy.node import Node
from sensor_msgs.msg import Image

from originbot_traffic_light._utils import clamp_roi as _clamp_roi
from originbot_traffic_light._utils import mask_ratio as _mask_ratio
from originbot_traffic_light._utils import order_corners as _order_corners

_DEBUG_THUMB_MAX_DIM = 160


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        # Topics
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', '/traffic_light/state')

        # Detection mode: "roi" (default, existing behaviour) or "card" (new auto mode)
        self.declare_parameter('mode', 'roi')

        # ROIs (absolute pixels on 640x480 by default)
        self.declare_parameter('left_x', 128)
        self.declare_parameter('left_y', 154)
        self.declare_parameter('left_w', 131)
        self.declare_parameter('left_h', 109)

        self.declare_parameter('mid_x', 259)
        self.declare_parameter('mid_y', 154)
        self.declare_parameter('mid_w', 131)
        self.declare_parameter('mid_h', 109)

        self.declare_parameter('right_x', 391)
        self.declare_parameter('right_y', 154)
        self.declare_parameter('right_w', 131)
        self.declare_parameter('right_h', 109)

        # HSV thresholds – green
        self.declare_parameter('green_h_low', 35)
        self.declare_parameter('green_s_low', 80)
        self.declare_parameter('green_v_low', 80)
        self.declare_parameter('green_h_high', 90)
        self.declare_parameter('green_s_high', 255)
        self.declare_parameter('green_v_high', 255)

        # HSV thresholds – red (two ranges to wrap around 0/180)
        self.declare_parameter('red1_h_low', 0)
        self.declare_parameter('red1_s_low', 80)
        self.declare_parameter('red1_v_low', 80)
        self.declare_parameter('red1_h_high', 10)
        self.declare_parameter('red1_s_high', 255)
        self.declare_parameter('red1_v_high', 255)

        self.declare_parameter('red2_h_low', 160)
        self.declare_parameter('red2_s_low', 80)
        self.declare_parameter('red2_v_low', 80)
        self.declare_parameter('red2_h_high', 179)
        self.declare_parameter('red2_s_high', 255)
        self.declare_parameter('red2_v_high', 255)

        # Ratio thresholds
        self.declare_parameter('min_green_ratio', 0.01)
        self.declare_parameter('min_red_ratio', 0.01)

        # Card-mode parameters
        self.declare_parameter('card_warp_width', 320)
        self.declare_parameter('card_warp_height', 120)
        self.declare_parameter('min_card_area', 5000)
        self.declare_parameter('card_aspect_ratio_min', 1.5)
        self.declare_parameter('card_aspect_ratio_max', 6.0)
        self.declare_parameter('band_margin_px', 5)

        # Always publish a state message on every processed frame, even when
        # detection fails or confidence is below threshold (state=UNKNOWN,
        # confidence=0.0).  Set publish_unknown:=false to revert to the old
        # behaviour of only publishing when a confident detection is made.
        self.declare_parameter('publish_unknown', True)

        # Debug image output (card mode)
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_topic', '/traffic_light/debug')

        # Cache HSV thresholds and ratio parameters (they rarely change)
        self.green_low = (
            int(self.get_parameter('green_h_low').value),
            int(self.get_parameter('green_s_low').value),
            int(self.get_parameter('green_v_low').value),
        )
        self.green_high = (
            int(self.get_parameter('green_h_high').value),
            int(self.get_parameter('green_s_high').value),
            int(self.get_parameter('green_v_high').value),
        )
        self.red1_low = (
            int(self.get_parameter('red1_h_low').value),
            int(self.get_parameter('red1_s_low').value),
            int(self.get_parameter('red1_v_low').value),
        )
        self.red1_high = (
            int(self.get_parameter('red1_h_high').value),
            int(self.get_parameter('red1_s_high').value),
            int(self.get_parameter('red1_v_high').value),
        )
        self.red2_low = (
            int(self.get_parameter('red2_h_low').value),
            int(self.get_parameter('red2_s_low').value),
            int(self.get_parameter('red2_v_low').value),
        )
        self.red2_high = (
            int(self.get_parameter('red2_h_high').value),
            int(self.get_parameter('red2_s_high').value),
            int(self.get_parameter('red2_v_high').value),
        )
        self.min_green = float(self.get_parameter('min_green_ratio').value)
        self.min_red = float(self.get_parameter('min_red_ratio').value)

        self.bridge = CvBridge()
        self.publish_unknown = bool(self.get_parameter('publish_unknown').value)

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.mode = self.get_parameter('mode').value

        self.sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self.pub = self.create_publisher(TrafficLight, output_topic, 10)

        # Optional debug publisher (card mode only)
        self.debug_pub = None
        if self.get_parameter('debug').value:
            debug_topic = self.get_parameter('debug_topic').value
            self.debug_pub = self.create_publisher(Image, debug_topic, 10)
            self.get_logger().info(f'Debug images on: {debug_topic}')

        self.get_logger().info(f'Mode: {self.mode}')
        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info(f'Publishing to:  {output_topic}')

    # ------------------------------------------------------------------
    # ROI mode helpers
    # ------------------------------------------------------------------

    def _roi_params(self, prefix, img_w, img_h):
        x = self.get_parameter(f'{prefix}_x').value
        y = self.get_parameter(f'{prefix}_y').value
        w = self.get_parameter(f'{prefix}_w').value
        h = self.get_parameter(f'{prefix}_h').value
        return _clamp_roi(x, y, w, h, img_w, img_h)

    def _eval_zone(self, hsv, prefix, img_w, img_h):
        x, y, w, h = self._roi_params(prefix, img_w, img_h)
        zone = hsv[y:y + h, x:x + w]

        green_mask = cv2.inRange(zone, self.green_low, self.green_high)
        red_mask = cv2.bitwise_or(
            cv2.inRange(zone, self.red1_low, self.red1_high),
            cv2.inRange(zone, self.red2_low, self.red2_high),
        )
        return _mask_ratio(green_mask), _mask_ratio(red_mask)

    # ------------------------------------------------------------------
    # Card mode helpers
    # ------------------------------------------------------------------

    def _find_card_quad(self, bgr):
        """Detect the largest plausible card quadrilateral in *bgr*.

        Returns a (4, 2) float32 array of corner points in ordered form, or
        None if no suitable contour is found.
        """
        min_area = int(self.get_parameter('min_card_area').value)
        ar_min = float(self.get_parameter('card_aspect_ratio_min').value)
        ar_max = float(self.get_parameter('card_aspect_ratio_max').value)

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        # Dilate edges slightly to close small gaps in the card border
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.dilate(edges, kernel, iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

            if len(approx) != 4:
                continue

            # Check aspect ratio using the bounding rect of the approximated quad
            x, y, w, h = cv2.boundingRect(approx)
            if h == 0:
                continue
            ar = float(w) / float(h)
            if not (ar_min <= ar <= ar_max):
                continue

            if area > best_area:
                best_area = area
                best = approx

        if best is None:
            return None
        return _order_corners(best)

    def _warp_card(self, bgr, corners):
        """Perspective-warp the detected card to a canonical rectangle."""
        w = int(self.get_parameter('card_warp_width').value)
        h = int(self.get_parameter('card_warp_height').value)
        dst = np.array(
            [[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]],
            dtype=np.float32,
        )
        M = cv2.getPerspectiveTransform(corners, dst)
        return cv2.warpPerspective(bgr, M, (w, h))

    def _classify_card(self, warped):
        """Return (state, confidence) from the warped card image."""
        margin = int(self.get_parameter('band_margin_px').value)
        h, w = warped.shape[:2]
        band_w = w // 3

        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.green_low, self.green_high)

        ratios = []
        for i in range(3):
            x0 = i * band_w + margin
            x1 = (i + 1) * band_w - margin
            x0 = max(0, x0)
            x1 = min(w, x1)
            if x1 <= x0:
                self.get_logger().warning(
                    f'band_margin_px ({margin}) is too large for band width '
                    f'({band_w}); band {i} will report 0 green ratio'
                )
                ratios.append(0.0)
                continue
            band = green_mask[:, x0:x1]
            ratios.append(_mask_ratio(band))

        max_ratio = max(ratios)
        if max_ratio < self.min_green:
            return TrafficLight.STOP, 0.0

        winner = ratios.index(max_ratio)
        confidence = float(max_ratio)
        if winner == 0:
            return TrafficLight.LEFT, confidence
        elif winner == 1:
            return TrafficLight.STRAIGHT, confidence
        else:
            return TrafficLight.RIGHT, confidence

    def _publish_debug(self, bgr, corners, warped, msg_header):
        """Publish a debug image: original frame with drawn card outline."""
        if self.debug_pub is None:
            return
        dbg = bgr.copy()
        if corners is not None:
            pts = corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(dbg, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
            # Overlay small warped card in top-left corner for reference
            if warped is not None:
                th, tw = warped.shape[:2]
                max_dim = _DEBUG_THUMB_MAX_DIM
                scale = min(max_dim / tw, max_dim / th)
                rw = max(1, int(tw * scale))
                rh = max(1, int(th * scale))
                thumb = cv2.resize(warped, (rw, rh))
                dbg[0:rh, 0:rw] = thumb
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            debug_msg.header = msg_header
            self.debug_pub.publish(debug_msg)
        except Exception as exc:
            self.get_logger().warning(f'Debug image publish failed: {exc}')

    # ------------------------------------------------------------------
    # Unified image callback
    # ------------------------------------------------------------------

    def _on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'cv_bridge conversion failed: {exc}')
            return

        # Build output message; state defaults to UNKNOWN / confidence 0.0
        # so downstream nodes always receive a heartbeat even when detection
        # fails or confidence is below threshold.
        out = TrafficLight()
        out.header = msg.header

        if self.mode == 'card':
            self._process_card_mode(bgr, msg.header, out)
        else:
            self._process_roi_mode(bgr, out)

        # Publish on every frame unless the user opted into the legacy
        # "only publish on confident detection" behaviour via publish_unknown=false.
        if self.publish_unknown or out.state != TrafficLight.UNKNOWN:
            self.pub.publish(out)

    def _process_roi_mode(self, bgr, out):
        img_h, img_w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        g_l, r_l = self._eval_zone(hsv, 'left', img_w, img_h)
        g_m, r_m = self._eval_zone(hsv, 'mid', img_w, img_h)
        g_r, r_r = self._eval_zone(hsv, 'right', img_w, img_h)

        greens = [g_l, g_m, g_r]
        reds = [r_l, r_m, r_r]
        green_on = [g > self.min_green for g in greens]
        red_on = [r > self.min_red for r in reds]

        if sum(green_on) == 1:
            # Exactly one lane is green – determine direction
            out.confidence = float(max(greens))
            if green_on[0]:
                out.state = TrafficLight.LEFT
            elif green_on[1]:
                out.state = TrafficLight.STRAIGHT
            else:
                out.state = TrafficLight.RIGHT
        elif sum(green_on) == 0 and all(red_on):
            # No green anywhere, all zones red – stop
            out.state = TrafficLight.STOP
            out.confidence = float(min(reds))
        else:
            out.state = TrafficLight.UNKNOWN
            out.confidence = 0.0

    def _process_card_mode(self, bgr, header, out):
        corners = self._find_card_quad(bgr)
        if corners is None:
            out.state = TrafficLight.UNKNOWN
            out.confidence = 0.0
            self._publish_debug(bgr, None, None, header)
            return

        warped = self._warp_card(bgr, corners)
        state, confidence = self._classify_card(warped)
        out.state = state
        out.confidence = confidence
        self._publish_debug(bgr, corners, warped, header)


def main():
    rclpy.init()
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
