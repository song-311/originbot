"""Intersection Action Manager for OriginBot.

Subscribes to /detect/traffic_decision (TrafficDecision) and translates
stable, high-confidence traffic-light decisions into movement commands
published on /control/moving/state (MovingParam).

State machine
-------------
IDLE
  │  first TrafficDecision message received
  ▼
APPROACH
  │  (immediate – no distance sensor in this package)
  ▼
WAIT_PERMISSION
  │  stable_frames consecutive frames of LEFT/RIGHT/STRAIGHT
  │  all with confidence ≥ conf_th
  ▼
EXECUTE_MANEUVER
  │  /control/moving/complete received (Bool, data=True)
  │  OR execute_timeout exceeded
  ▼
DONE
  │  done_reset_delay elapsed
  ▼
IDLE

STOP and UNKNOWN keep the node in WAIT_PERMISSION (no movement command).
"""

import enum
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from originbot_msgs.msg import MovingParam, TrafficDecision

# Map TrafficDecision states → MovingParam moving_type
_DECISION_TO_MOVING = {
    TrafficDecision.STRAIGHT: MovingParam.MOVING_FORWARD,
    TrafficDecision.LEFT: MovingParam.MOVING_LEFT,
    TrafficDecision.RIGHT: MovingParam.MOVING_RIGHT,
    TrafficDecision.STOP: MovingParam.MOVING_STOP,
}

_MOVING_NAMES = {
    MovingParam.MOVING_UNKNOWN: 'UNKNOWN',
    MovingParam.MOVING_STOP: 'STOP',
    MovingParam.MOVING_FORWARD: 'STRAIGHT',
    MovingParam.MOVING_LEFT: 'LEFT',
    MovingParam.MOVING_RIGHT: 'RIGHT',
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


class IntersectionActionManager(Node):
    """ROS 2 node that manages intersection actions based on traffic decisions."""

    def __init__(self):
        super().__init__('intersection_action_manager')

        # ── Parameters ────────────────────────────────────────────────────────
        # Minimum confidence to treat a detection as valid; below this value
        # the state is treated as UNKNOWN (conservative).
        self.declare_parameter('conf_th', 0.6)

        # Number of consecutive frames the same actionable state must be seen
        # before a movement command is issued.
        self.declare_parameter('stable_frames', 3)

        # Seconds to wait in WAIT_PERMISSION before emitting a timeout warning
        # and resetting the stable-frame counter.  Does NOT force movement.
        self.declare_parameter('wait_timeout', 30.0)

        # Seconds before EXECUTE_MANEUVER is forcibly exited with an error log.
        self.declare_parameter('execute_timeout', 10.0)

        # Seconds in DONE state before auto-resetting to IDLE for the next
        # intersection.
        self.declare_parameter('done_reset_delay', 3.0)

        # Maximum velocity published to /control/max_vel while approaching.
        self.declare_parameter('max_vel_approach', 0.1)

        # Maximum velocity published to /control/max_vel during execution.
        self.declare_parameter('max_vel_execute', 0.15)

        self._conf_th = float(self.get_parameter('conf_th').value)
        self._stable_frames = int(self.get_parameter('stable_frames').value)
        self._wait_timeout = float(self.get_parameter('wait_timeout').value)
        self._execute_timeout = float(self.get_parameter('execute_timeout').value)
        self._done_reset_delay = float(self.get_parameter('done_reset_delay').value)
        self._max_vel_approach = float(self.get_parameter('max_vel_approach').value)
        self._max_vel_execute = float(self.get_parameter('max_vel_execute').value)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            TrafficDecision,
            '/detect/traffic_decision',
            self._on_traffic_decision,
            10,
        )
        self.create_subscription(
            Bool,
            '/control/moving/complete',
            self._on_moving_complete,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_moving_state = self.create_publisher(
            MovingParam, '/control/moving/state', 10
        )
        self._pub_max_vel = self.create_publisher(Float64, '/control/max_vel', 10)

        # ── Internal state ────────────────────────────────────────────────────
        self._state = _State.IDLE
        # Ring buffer holding the last stable_frames filtered decision states
        self._decision_buffer: deque = deque(maxlen=self._stable_frames)
        self._wait_start = None
        self._execute_start = None
        self._done_time = None
        # Guard: only allow one movement command per EXECUTE_MANEUVER phase
        self._maneuver_sent = False

        # Periodic 10 Hz timer used for timeout enforcement
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'IntersectionActionManager ready  '
            f'(conf_th={self._conf_th}, stable_frames={self._stable_frames}, '
            f'wait_timeout={self._wait_timeout}s)'
        )

    # ── Topic callbacks ───────────────────────────────────────────────────────

    def _on_traffic_decision(self, msg: TrafficDecision) -> None:
        # Apply confidence threshold: low-confidence detections are UNKNOWN
        filtered = msg.state if msg.confidence >= self._conf_th else TrafficDecision.UNKNOWN

        if self._state == _State.IDLE:
            # First message: start approach phase
            self._transition(_State.APPROACH)
            self._decision_buffer.clear()
            self._decision_buffer.append(filtered)
            # APPROACH is a pass-through state (no distance sensor available);
            # proceed immediately to WAIT_PERMISSION.
            self._transition(_State.WAIT_PERMISSION)

        elif self._state in (_State.APPROACH, _State.WAIT_PERMISSION):
            self._decision_buffer.append(filtered)
            self._check_permission()

        # Messages received in EXECUTE_MANEUVER / DONE are intentionally ignored
        # to prevent re-triggering while an action is in progress.

    def _on_moving_complete(self, msg: Bool) -> None:
        if self._state == _State.EXECUTE_MANEUVER and msg.data:
            self.get_logger().info('Maneuver complete – /control/moving/complete received')
            self._transition(_State.DONE)

    # ── Permission check ──────────────────────────────────────────────────────

    def _check_permission(self) -> None:
        """Issue a movement command when the buffer is full of one actionable state."""
        if len(self._decision_buffer) < self._stable_frames:
            return

        recent = list(self._decision_buffer)[-self._stable_frames:]
        candidate = recent[0]

        # STOP and UNKNOWN do not permit through-movement
        if candidate in (TrafficDecision.UNKNOWN, TrafficDecision.STOP):
            return

        if all(s == candidate for s in recent):
            self._execute_maneuver(candidate)

    # ── Maneuver execution ────────────────────────────────────────────────────

    def _execute_maneuver(self, direction: int) -> None:
        if self._maneuver_sent:
            return  # Prevent double-trigger within the same phase

        moving_type = _DECISION_TO_MOVING.get(direction, MovingParam.MOVING_UNKNOWN)

        msg = MovingParam()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.moving_type = moving_type
        # moving_value = 0.0: let the downstream executor decide the
        # distance/angle; the direction is encoded in moving_type.
        msg.moving_value = 0.0

        self._pub_moving_state.publish(msg)
        self._maneuver_sent = True

        direction_name = _DECISION_NAMES.get(direction, 'UNKNOWN')
        self.get_logger().info(
            f'Maneuver triggered: {direction_name}  '
            f'(moving_type={_MOVING_NAMES.get(moving_type, "?")})'
        )

        # Raise speed limit for execution phase
        self._publish_max_vel(self._max_vel_execute)
        self._transition(_State.EXECUTE_MANEUVER)

    # ── Timeout enforcement (10 Hz timer) ─────────────────────────────────────

    def _tick(self) -> None:
        now = self.get_clock().now()

        if self._state == _State.WAIT_PERMISSION and self._wait_start is not None:
            elapsed = (now - self._wait_start).nanoseconds * 1e-9
            if elapsed > self._wait_timeout:
                self.get_logger().warning(
                    f'WAIT_PERMISSION timeout ({self._wait_timeout:.0f}s) reached. '
                    'Resetting stable-frame counter and holding position '
                    '(conservative strategy).'
                )
                # Reset the buffer and restart the wait timer so the node
                # keeps watching for a valid signal without forcing movement.
                self._decision_buffer.clear()
                self._wait_start = now
                # Publish zero velocity limit as conservative safety measure
                self._publish_max_vel(0.0)

        elif self._state == _State.EXECUTE_MANEUVER and self._execute_start is not None:
            elapsed = (now - self._execute_start).nanoseconds * 1e-9
            if elapsed > self._execute_timeout:
                self.get_logger().error(
                    f'EXECUTE_MANEUVER timeout ({self._execute_timeout:.0f}s) reached. '
                    'Forcing transition to DONE to avoid deadlock.'
                )
                self._transition(_State.DONE)

        elif self._state == _State.DONE and self._done_time is not None:
            elapsed = (now - self._done_time).nanoseconds * 1e-9
            if elapsed > self._done_reset_delay:
                self.get_logger().info(
                    f'DONE reset delay ({self._done_reset_delay:.0f}s) elapsed. '
                    'Returning to IDLE for next intersection.'
                )
                self._transition(_State.IDLE)

    # ── State transition helper ───────────────────────────────────────────────

    def _transition(self, new_state: _State) -> None:
        old_state = self._state
        self._state = new_state
        now = self.get_clock().now()

        if new_state == _State.APPROACH:
            # Begin reducing speed for intersection approach
            self._publish_max_vel(self._max_vel_approach)

        elif new_state == _State.WAIT_PERMISSION:
            self._wait_start = now
            self._maneuver_sent = False
            # Ensure approach speed is in effect while waiting
            self._publish_max_vel(self._max_vel_approach)

        elif new_state == _State.EXECUTE_MANEUVER:
            self._execute_start = now

        elif new_state == _State.DONE:
            self._done_time = now

        elif new_state == _State.IDLE:
            self._decision_buffer.clear()
            self._wait_start = None
            self._execute_start = None
            self._done_time = None
            self._maneuver_sent = False

        self.get_logger().info(
            f'FSM: {old_state.value} → {new_state.value}'
        )

    # ── Utility ───────────────────────────────────────────────────────────────

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
