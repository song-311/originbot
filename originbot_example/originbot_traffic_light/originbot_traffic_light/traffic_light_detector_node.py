import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from originbot_msgs.msg import TrafficLight
from rclpy.node import Node
from sensor_msgs.msg import Image


def _clamp_roi(x, y, w, h, img_w, img_h):
    x = max(0, min(int(x), img_w - 1))
    y = max(0, min(int(y), img_h - 1))
    w = max(1, min(int(w), img_w - x))
    h = max(1, min(int(h), img_h - y))
    return x, y, w, h


def _mask_ratio(mask):
    return float(np.count_nonzero(mask)) / float(mask.size)


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        # Topics
        self.declare_parameter('image_topic', '/bgr8_image')
        self.declare_parameter('output_topic', '/traffic_light/state')

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

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self.pub = self.create_publisher(TrafficLight, output_topic, 10)

        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info(f'Publishing to:  {output_topic}')

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

    def _on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'cv_bridge conversion failed: {exc}')
            return

        img_h, img_w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        g_l, r_l = self._eval_zone(hsv, 'left', img_w, img_h)
        g_m, r_m = self._eval_zone(hsv, 'mid', img_w, img_h)
        g_r, r_r = self._eval_zone(hsv, 'right', img_w, img_h)

        greens = [g_l, g_m, g_r]
        reds = [r_l, r_m, r_r]
        green_on = [g > self.min_green for g in greens]
        red_on = [r > self.min_red for r in reds]

        out = TrafficLight()
        out.header = msg.header

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

        self.pub.publish(out)


def main():
    rclpy.init()
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
