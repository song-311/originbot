"""Intersection Action Manager for OriginBot.

Subscribes to /detect/traffic_decision (TrafficDecision) and translates
stable, high-confidence traffic-light decisions into movement commands
published on /control/moving/state (MovingParam).

Updated policy:
- STOP: publish MOVING_STOP and hold max_vel=0.0 (once per STOP phase)
- LEFT/RIGHT: execute turn with yaw angle constraints (primary)
  and optional line-reacquire early exit
- Timeout is fallback only
"""

import enum
import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from originbot_msgs.msg import MovingParam, TrafficDecision


_DECISION_TO_MOVING = {
    TrafficDecision.STRAIGHT: MovingParam.MOVING_FORWARD,
    TrafficDecision.LEFT: MovingParam.MOVING_LEFT,
    TrafficDecision.RIGHT: MovingParam.MOVING_RIGHT,
    TrafficDecision.STOP: MovingParam.MOVING_STOP,
}

_DECISION_NAMES = {
    TrafficDecision.UNKNOWN: 'UNKNOWN',
    TrafficDecision.STOP: 'STOP',
    TrafficDecision.STRAIGHT: 'STRAIGHT',
    TrafficDecision.LEFT: 'LEFT',
    TrafficDecision.RIGHT: 'RIGHT',
}


class _State(enum.Enum):
    IDLE = 'IDLE'
    APPROACH = 'APPROACH'
    WAIT_PERMISSION = 'WAIT_PERMISSION'
    EXECUTE_MANEUVER = 'EXECUTE_MANEUVER'
    DONE = 'DONE'


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # ROS quaternion -> yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class IntersectionActionManager(Node):
    def __init__(self):
        super().__init__('intersection_action_manager')

        # ===== parameters =====
        self.declare_parameter('conf_th', 0.6)
        self.declare_parameter('stable_frames', 3)
        self.declare_parameter('wait_timeout', 30.0)
        self.declare_parameter('execute_timeout', 10.0)
        self.declare_parameter('done_reset_delay', 3.0)
        self.declare_parameter('max_vel_approach', 0.1)
        self.declare_parameter('max_vel_execute', 0.15)

        # new: angle-driven turn control
        self.declare_parameter('turn_target_angle_deg', 90.0)
        self.declare_parameter('turn_max_angle_deg', 105.0)
        self.declare_parameter('turn_min_angle_deg_for_line_exit', 55.0)
        self.declare_parameter('right_turn_negative', True)

        # new: line reacquire optional early-exit
        self.declare_parameter('turn_use_line_reacquire', True)
        self.declare_parameter('line_reacquire_topic', '/line_follow/line_found')
        self.declare_parameter('line_reacquire_true_frames', 2)

        # new: odom topic for yaw
        self.declare_parameter('odom_topic', '/odom')

        self._conf_th = float(self.get_parameter('conf_th').value)
        self._stable_frames = int(self.get_parameter('stable_frames').value)
        self._wait_timeout = float(self.get_parameter('wait_timeout').value)
        self._execute_timeout = float(self.get_parameter('execute_timeout').value)
        self._done_reset_delay = float(self.get_parameter('done_reset_delay').value)
        self._max_vel_approach = float(self.get_parameter('max_vel_approach').value)
        self._max_vel_execute = float(self.get_parameter('max_vel_execute').value)

        self._turn_target_angle_deg = float(self.get_parameter('turn_target_angle_deg').value)
        self._turn_max_angle_deg = float(self.get_parameter('turn_max_angle_deg').value)
        self._turn_min_angle_deg_for_line_exit = float(
            self.get_parameter('turn_min_angle_deg_for_line_exit').value
        )
        self._right_turn_negative = bool(self.get_parameter('right_turn_negative').value)
        self._turn_use_line_reacquire = bool(self.get_parameter('turn_use_line_reacquire').value)
        self._line_reacquire_topic = str(self.get_parameter('line_reacquire_topic').value)
        self._line_reacquire_true_frames = int(self.get_parameter('line_reacquire_true_frames').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)

        # ===== subscriptions =====
        self.create_subscription(TrafficDecision, '/detect/traffic_decision', self._on_traffic_decision, 10)
        self.create_subscription(Bool, '/control/moving/complete', self._on_moving_complete, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 30)
        self.create_subscription(Bool, self._line_reacquire_topic, self._on_line_found, 30)

        # ===== publishers =====
        self._pub_moving_state = self.create_publisher(MovingParam, '/control/moving/state', 10)
        self._pub_max_vel = self.create_publisher(Float64, '/control/max_vel', 10)

        # ===== internal =====
        self._state = _State.IDLE
        self._decision_buffer: deque = deque(maxlen=self._stable_frames)
        self._wait_start = None
        self._execute_start = None
        self._done_time = None

        self._maneuver_sent = False
        self._stop_sent = False

        self._current_decision = TrafficDecision.UNKNOWN

        # turn tracking
        self._current_yaw = None
        self._turn_start_yaw = None
        self._turn_direction = TrafficDecision.UNKNOWN  # LEFT/RIGHT/STRAIGHT
        self._line_true_count = 0
        self._line_found = False

        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            'IntersectionActionManager ready '
            f'(conf_th={self._conf_th}, stable_frames={self._stable_frames}, '
            f'turn_target={self._turn_target_angle_deg}deg, turn_max={self._turn_max_angle_deg}deg)'
        )

    # ===== callbacks =====
    def _on_traffic_decision(self, msg: TrafficDecision) -> None:
        filtered = msg.state if msg.confidence >= self._conf_th else TrafficDecision.UNKNOWN

        if self._state == _State.IDLE:
            self._transition(_State.APPROACH)
            self._decision_buffer.clear()
            self._decision_buffer.append(filtered)
            self._transition(_State.WAIT_PERMISSION)

        elif self._state in (_State.APPROACH, _State.WAIT_PERMISSION):
            self._decision_buffer.append(filtered)
            self._check_permission()

    def _on_moving_complete(self, msg: Bool) -> None:
        # keep compatibility with existing executor callback
        if self._state == _State.EXECUTE_MANEUVER and msg.data:
            self.get_logger().info('Maneuver complete from /control/moving/complete')
            self._transition(_State.DONE)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._current_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

    def _on_line_found(self, msg: Bool) -> None:
        self._line_found = bool(msg.data)
        if self._line_found:
            self._line_true_count += 1
        else:
            self._line_true_count = 0

    # ===== decision logic =====
    def _check_permission(self) -> None:
        if len(self._decision_buffer) < self._stable_frames:
            return

        recent = list(self._decision_buffer)[-self._stable_frames:]
        candidate = recent[0]
        if not all(s == candidate for s in recent):
            return

        # UNKNOWN: keep waiting
        if candidate == TrafficDecision.UNKNOWN:
            return

        # STOP: actively hold stop
        if candidate == TrafficDecision.STOP:
            self._execute_stop()
            return

        # STRAIGHT / LEFT / RIGHT
        self._execute_maneuver(candidate)

    def _execute_stop(self) -> None:
        if self._stop_sent:
            return

        msg = MovingParam()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.moving_type = MovingParam.MOVING_STOP
        msg.moving_value = 0.0
        self._pub_moving_state.publish(msg)

        self._publish_max_vel(0.0)
        self._stop_sent = True
        self._maneuver_sent = False
        self._current_decision = TrafficDecision.STOP

        self.get_logger().info('STOP triggered: MOVING_STOP published, max_vel=0.0')

    def _execute_maneuver(self, direction: int) -> None:
        if self._maneuver_sent:
            return

        moving_type = _DECISION_TO_MOVING.get(direction, MovingParam.MOVING_UNKNOWN)

        msg = MovingParam()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.moving_type = moving_type

        # keep executor compatibility:
        # - STRAIGHT: 0.0 (executor decides distance)
        # - LEFT/RIGHT: target angle in rad (sign per configuration)
        if direction == TrafficDecision.LEFT:
            msg.moving_value = math.radians(self._turn_target_angle_deg)
        elif direction == TrafficDecision.RIGHT:
            angle = math.radians(self._turn_target_angle_deg)
            msg.moving_value = -angle if self._right_turn_negative else angle
        else:
            msg.moving_value = 0.0

        self._pub_moving_state.publish(msg)
        self._maneuver_sent = True
        self._stop_sent = False
        self._current_decision = direction

        # set turn tracking for LEFT/RIGHT
        self._turn_direction = direction
        self._turn_start_yaw = self._current_yaw
        self._line_true_count = 0

        self._publish_max_vel(self._max_vel_execute)
        self._transition(_State.EXECUTE_MANEUVER)

        self.get_logger().info(
            f'Maneuver start: {_DECISION_NAMES.get(direction, "?")} '
            f'(moving_type={moving_type}, moving_value={msg.moving_value:.4f})'
        )

    # ===== periodic tick =====
    def _tick(self) -> None:
        now = self.get_clock().now()

        if self._state == _State.WAIT_PERMISSION and self._wait_start is not None:
            elapsed = (now - self._wait_start).nanoseconds * 1e-9
            if elapsed > self._wait_timeout:
                self.get_logger().warning(
                    f'WAIT_PERMISSION timeout ({self._wait_timeout:.1f}s), reset buffer and hold.'
                )
                self._decision_buffer.clear()
                self._wait_start = now
                self._publish_max_vel(0.0)

        elif self._state == _State.EXECUTE_MANEUVER and self._execute_start is not None:
            # NEW: angle-dominant termination for LEFT/RIGHT
            # STRAIGHT relies on the fallback timeout below
            if self._turn_direction in (TrafficDecision.LEFT, TrafficDecision.RIGHT):
                if self._current_yaw is not None and self._turn_start_yaw is not None:
                    delta = _normalize_angle(self._current_yaw - self._turn_start_yaw)
                    turned_deg = abs(math.degrees(delta))

                    # 1) optional early exit by line reacquire (after min turn angle)
                    if self._turn_use_line_reacquire:
                        if (
                            turned_deg >= self._turn_min_angle_deg_for_line_exit
                            and self._line_true_count >= self._line_reacquire_true_frames
                        ):
                            self.get_logger().info(
                                f'Turn completed: line reacquired after {turned_deg:.1f} deg'
                            )
                            self._transition(_State.DONE)
                            return

                    # 2) hard safety cap by max angle
                    if turned_deg >= self._turn_max_angle_deg:
                        self.get_logger().warning(
                            f'Turn capped by max angle ({turned_deg:.1f} >= {self._turn_max_angle_deg:.1f})'
                        )
                        self._transition(_State.DONE)
                        return

            # 3) fallback timeout
            elapsed = (now - self._execute_start).nanoseconds * 1e-9
            if elapsed > self._execute_timeout:
                self.get_logger().error(
                    f'EXECUTE timeout ({self._execute_timeout:.1f}s), forcing DONE'
                )
                self._transition(_State.DONE)

        elif self._state == _State.DONE and self._done_time is not None:
            elapsed = (now - self._done_time).nanoseconds * 1e-9
            if elapsed > self._done_reset_delay:
                self._transition(_State.IDLE)

    # ===== transitions/util =====
    def _transition(self, new_state: _State) -> None:
        old_state = self._state
        self._state = new_state
        now = self.get_clock().now()

        if new_state == _State.APPROACH:
            self._publish_max_vel(self._max_vel_approach)

        elif new_state == _State.WAIT_PERMISSION:
            self._wait_start = now
            self._maneuver_sent = False
            self._stop_sent = False
            self._decision_buffer.clear()
            self._publish_max_vel(self._max_vel_approach)

        elif new_state == _State.EXECUTE_MANEUVER:
            self._execute_start = now

        elif new_state == _State.DONE:
            self._done_time = now
            self._publish_max_vel(0.0)

        elif new_state == _State.IDLE:
            self._decision_buffer.clear()
            self._wait_start = None
            self._execute_start = None
            self._done_time = None
            self._maneuver_sent = False
            self._stop_sent = False
            self._current_decision = TrafficDecision.UNKNOWN
            self._turn_direction = TrafficDecision.UNKNOWN
            self._turn_start_yaw = None
            self._line_true_count = 0

        self.get_logger().info(f'FSM: {old_state.value} -> {new_state.value}')

    def _publish_max_vel(self, vel: float) -> None:
        msg = Float64()
        msg.data = float(vel)
        self._pub_max_vel.publish(msg)


def main():
    rclpy.init()
    node = IntersectionActionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
