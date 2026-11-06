"""circle_tracker: drives the robot round a circle with full-state feedback (pole placement).
    ros2 launch sparcbot_bringup robot.launch.py            # terminal 1
    ros2 launch sparcbot_bringup circle_tracker.launch.py   # terminal 2
    ros2 service call /circle_tracker/start std_srvs/srv/Trigger   # terminal 3

Every /odom message while running:
  1. find where the reference point should be now (time since ~/start)
  2. error e = reference - robot, expressed in the robot frame
  3. publish /cmd_vel
"""

import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_srvs.srv import Trigger


# ======================================================================
# SETTINGS: everything you may want to change is here (nothing is in sparcbot.yaml)
# ======================================================================

# The circle
CIRCLE_RADIUS_M = 0.4           # Lab 6 radius [m]
SPEED_M_S = 0.2                 # Lab 6 forward speed [m/s]
COUNTER_CLOCKWISE = True        # False = curve to the right
LAPS = 1.0
AUTOSTART = False               # True = drive as soon as /odom arrives (leave False)

# Lab 6 pole-placement gains (for v_ref=0.2, w_ref=+0.5).
#TODO Set your gains here
K1 = 0.0
K5 = 0.0
K6 = 0.0

# Safety
MAX_LINEAR_M_S = 0.30           # command limits
MAX_ANGULAR_RAD_S = 2.0
MAX_POSITION_ERROR_M = 0.30     # stop if the robot is further than this from the reference
ODOM_TIMEOUT_S = 0.5            # stop if /odom is silent for this long


# Math
def wrap_angle(angle_rad: float) -> float:
    """Wrap to [-pi, pi]."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


# ----------------------------------------------------------------------
# Pole-placement gain
# ----------------------------------------------------------------------
def pole_placement_gain(v_ref: float, w_ref: float) -> np.ndarray:
    """Return a structured K placing poles at -1, -2, -3.

    Lab 6 provides gains for v=0.2 m/s and w=+0.5 rad/s.
    For clockwise motion, reflect the lateral error and yaw channel.
    Other reference speeds are rejected.
    """
    if not math.isclose(v_ref, 0.2, abs_tol=1e-9) or not math.isclose(abs(w_ref), 0.5, abs_tol=1e-9):
        raise ValueError('Lab 6 gains require v_ref=0.2 m/s and |w_ref|=0.5 rad/s')
    side = 1.0 if w_ref > 0 else -1.0
    return np.array([[K1, 0.0, 0.0], [0.0, side * K5, K6]])


def circle_error_model(v_ref: float, w_ref: float):
    """(A, B) of the linearised tracking-error dynamics."""
    A = np.array([
        [0.0, w_ref, 0.0],
        [-w_ref, 0.0, v_ref],
        [0.0, 0.0, 0.0],
    ])
    B = np.array([
        [-1.0, 0.0],
        [0.0, 0.0],
        [0.0, -1.0],
    ])
    return A, B


# ----------------------------------------------------------------------
# Reference path
# ----------------------------------------------------------------------

@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class ReferencePoint:
    pose: Pose
    v_m_s: float        # feedforward linear speed
    w_rad_s: float      # feedforward turn rate


class CircleReference:
    """
        counter-clockwise:  centre is 1 radius to the robot's LEFT
        clockwise:          centre is 1 radius to the robot's RIGHT
    """

    def __init__(self, start: Pose, radius_m: float, speed_m_s: float,
                 counter_clockwise: bool = True, laps: float = 1.0):
        if radius_m <= 0.0:
            raise ValueError('radius_m must be > 0')
        if speed_m_s <= 0.0:
            raise ValueError('speed_m_s must be > 0')
        if laps < 0.0:
            raise ValueError('laps must be >= 0 (0 = never finish)')

        self.start = start
        self.radius_m = radius_m
        self.speed_m_s = speed_m_s
        self.side = 1.0 if counter_clockwise else -1.0
        self.w_rad_s = self.side * speed_m_s / radius_m
        self.laps = laps

        # Time for the requested number of laps (infinite if laps == 0).
        self.duration_s = (
            math.inf if laps == 0.0
            else laps * 2.0 * math.pi * radius_m / speed_m_s)

    def finished(self, t_s: float) -> bool:
        return t_s >= self.duration_s

    def at(self, t_s: float) -> ReferencePoint:
        t_s = max(0.0, min(t_s, self.duration_s))
        phi = self.w_rad_s * t_s

        # Circle in the start frame.
        x_l = self.side * self.radius_m * math.sin(phi)
        y_l = self.side * self.radius_m * (1.0 - math.cos(phi))
        yaw_l = phi

        # Rotate/translate into the odom frame.
        c = math.cos(self.start.yaw)
        s = math.sin(self.start.yaw)
        pose = Pose(
            x=self.start.x + c * x_l - s * y_l,
            y=self.start.y + s * x_l + c * y_l,
            yaw=wrap_angle(self.start.yaw + yaw_l))

        return ReferencePoint(pose=pose, v_m_s=self.speed_m_s, w_rad_s=self.w_rad_s)


# ----------------------------------------------------------------------
# Controller
# ----------------------------------------------------------------------

@dataclass
class StateFeedbackConfig:
    # Command limits.
    max_linear_m_s: float = MAX_LINEAR_M_S
    max_angular_rad_s: float = MAX_ANGULAR_RAD_S

    def __post_init__(self):
        for name in ('max_linear_m_s', 'max_angular_rad_s'):
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be > 0')

@dataclass
class ControllerOutput:
    v_m_s: float
    w_rad_s: float
    error: tuple        # (e_x, e_y, e_theta) in the robot frame


class CircleStateFeedback:
    """u = u_ref - K e, with K from Lab 6 pole placement."""

    def __init__(self, v_ref_m_s: float, w_ref_rad_s: float, config: StateFeedbackConfig):
        self.config = config
        self.v_ref = v_ref_m_s
        self.w_ref = w_ref_rad_s

        A, B = circle_error_model(v_ref_m_s, w_ref_rad_s)
        self.A, self.B = A, B
        self.K = pole_placement_gain(v_ref_m_s, w_ref_rad_s)

    def closed_loop_poles(self) -> np.ndarray:
        return np.linalg.eigvals(self.A - self.B @ self.K)

    @staticmethod
    def tracking_error(robot: Pose, ref: Pose) -> np.ndarray:
        """Reference minus robot, expressed in the robot frame."""
        dx = ref.x - robot.x
        dy = ref.y - robot.y
        c = math.cos(robot.yaw)
        s = math.sin(robot.yaw)
        return np.array([
            c * dx + s * dy,
            -s * dx + c * dy,
            wrap_angle(ref.yaw - robot.yaw),
        ])

    def command(self, robot: Pose, ref: ReferencePoint) -> ControllerOutput:
        e = self.tracking_error(robot, ref.pose)

        du = -self.K @ e
        v = du[0] + ref.v_m_s
        w = du[1] + ref.w_rad_s

        v = max(-self.config.max_linear_m_s, min(self.config.max_linear_m_s, v))
        w = max(-self.config.max_angular_rad_s, min(self.config.max_angular_rad_s, w))

        return ControllerOutput(v_m_s=v, w_rad_s=w, error=(e[0], e[1], e[2]))


# ======================================================================
# ROS node
# ======================================================================

IDLE = 'idle'
START_PENDING = 'start_pending'
RUNNING = 'running'


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CircleTrackerNode(Node):

    def __init__(self):
        super().__init__('circle_tracker')

        self.radius_m = CIRCLE_RADIUS_M
        self.speed_m_s = SPEED_M_S
        self.counter_clockwise = COUNTER_CLOCKWISE
        self.laps = LAPS
        self.autostart = AUTOSTART
        self.max_position_error_m = MAX_POSITION_ERROR_M
        self.odom_timeout_s = ODOM_TIMEOUT_S

        self.config = StateFeedbackConfig()   # command limits from SETTINGS

        # Fail at start-up (not mid-lap) if the settings are not usable.
        CircleReference(Pose(), self.radius_m, self.speed_m_s, self.counter_clockwise, self.laps)
        preview = CircleStateFeedback(
            self.speed_m_s,
            (1.0 if self.counter_clockwise else -1.0) * self.speed_m_s / self.radius_m,
            self.config)

        self.state = IDLE
        self.reference = None
        self.controller = None
        self.original_start_pose = None
        self.t0_s = 0.0
        self.last_odom_time = None
        self._autostart_done = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ref_pub = self.create_publisher(PoseStamped, '~/reference', 10)
        self.error_pub = self.create_publisher(Vector3Stamped, '~/tracking_error', 10)

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(0.1, self.on_watchdog)

        self.get_logger().info(
            f'circle_tracker ready: circle {2.0 * self.radius_m:.2f} m across at '
            f'{self.speed_m_s:.2f} m/s, '
            f'{"counter-clockwise" if self.counter_clockwise else "clockwise"}, '
            f'{"until stopped" if self.laps == 0.0 else f"{self.laps:g} lap(s)"}.')
        self.get_logger().info(
            'Feedback gain K (rows: v, w; columns: e_x, e_y, e_theta):\n'
            + '\n'.join('  ' + '  '.join(f'{k:8.4f}' for k in row) for row in preview.K))
        self.get_logger().info(
            'Closed-loop poles: '
            + ', '.join(f'{p.real:.2f}' for p in preview.closed_loop_poles()))
        if self.autostart:
            self.get_logger().warn('autostart is on: the robot drives as soon as /odom arrives.')
        else:
            self.get_logger().info('Idle. Call ~/start to begin.')

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def on_start(self, request, response):
        if self.state == RUNNING:
            response.success = False
            response.message = 'Already running. Call ~/stop first.'
        elif self.last_odom_time is None:
            self.state = START_PENDING
            response.success = True
            response.message = 'No /odom yet: will start as soon as it arrives.'
        else:
            self.state = START_PENDING
            response.success = True
            response.message = 'Starting from the next /odom.'
        return response

    def on_stop(self, request, response):
        self.stop_robot('Stopped by ~/stop.')
        response.success = True
        response.message = 'Stopped.'
        return response

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def on_odom(self, msg: Odometry) -> None:
        self.last_odom_time = self.get_clock().now()

        pose = Pose(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw_from_quaternion(msg.pose.pose.orientation))
        stamp_s = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        if self.state == IDLE and self.autostart and not self._autostart_done:
            self._autostart_done = True
            self.state = START_PENDING

        if self.state == START_PENDING:
            self._begin(pose, stamp_s)

        if self.state != RUNNING:
            return

        t_s = stamp_s - self.t0_s
        if self.reference.finished(t_s):
            final = self.controller.tracking_error(pose, self.reference.at(t_s).pose)
            self.stop_robot(
                f'Finished. Final position error {math.hypot(final[0], final[1]) * 100.0:.1f} cm, '
                f'heading error {math.degrees(final[2]):.1f} deg.')
            return

        ref = self.reference.at(t_s)
        out = self.controller.command(pose, ref)

        position_error_m = math.hypot(out.error[0], out.error[1])
        if position_error_m > self.max_position_error_m:
            self.stop_robot(
                f'ABORT: {position_error_m:.2f} m from the reference '
                f'(limit {self.max_position_error_m:.2f} m). Is a wheel blocked or reversed?',
                warn=True)
            return

        cmd = Twist()
        cmd.linear.x = float(out.v_m_s)
        cmd.angular.z = float(out.w_rad_s)
        self.cmd_pub.publish(cmd)

        self._publish_debug(msg, ref, out)

    def _begin(self, pose: Pose, stamp_s: float) -> None:

        # Save the original starting pose only once.
        if self.original_start_pose is None:
            self.original_start_pose = Pose(
                x=pose.x,
                y=pose.y,
                yaw=pose.yaw
            )

        # Always generate the same reference circle.
        self.reference = CircleReference(
            self.original_start_pose,
            self.radius_m,
            self.speed_m_s,
            self.counter_clockwise,
            self.laps
        )

        self.controller = CircleStateFeedback(
            self.reference.speed_m_s,
            self.reference.w_rad_s,
            self.config
        )

        # Reset trajectory time for each new lap.
        self.t0_s = stamp_s
        self.state = RUNNING

        self.get_logger().info(
            f'Actual start: x={pose.x:.3f}, '
            f'y={pose.y:.3f}, '
            f'yaw={math.degrees(pose.yaw):.1f} deg. '
            f'Original reference: '
            f'x={self.original_start_pose.x:.3f}, '
            f'y={self.original_start_pose.y:.3f}, '
            f'yaw={math.degrees(self.original_start_pose.yaw):.1f} deg.'
        )

    def _publish_debug(self, odom_msg, ref, out) -> None:
        ref_msg = PoseStamped()
        ref_msg.header = odom_msg.header
        ref_msg.pose.position.x = ref.pose.x
        ref_msg.pose.position.y = ref.pose.y
        ref_msg.pose.orientation.z = math.sin(0.5 * ref.pose.yaw)
        ref_msg.pose.orientation.w = math.cos(0.5 * ref.pose.yaw)
        self.ref_pub.publish(ref_msg)

        err_msg = Vector3Stamped()
        err_msg.header = odom_msg.header
        err_msg.vector.x = float(out.error[0])
        err_msg.vector.y = float(out.error[1])
        err_msg.vector.z = float(out.error[2])
        self.error_pub.publish(err_msg)

    def on_watchdog(self) -> None:
        if self.state != RUNNING or self.last_odom_time is None:
            return
        age_s = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if age_s > self.odom_timeout_s:
            self.stop_robot(f'ABORT: no /odom for {age_s:.2f} s.', warn=True)

    def stop_robot(self, reason: str, warn: bool = False) -> None:
        """Command zero velocity and go idle."""
        was_active = self.state != IDLE
        self.state = IDLE
        self.cmd_pub.publish(Twist())
        if was_active:
            (self.get_logger().warn if warn else self.get_logger().info)(reason)


def main(args=None):
    # Keep the ROS context alive on Ctrl-C so the final zero /cmd_vel can be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CircleTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # wheel_controller holds the last /cmd_vel, so if we were driving,
            # leave a zero behind.
            if node.state != IDLE:
                for _ in range(3):
                    node.cmd_pub.publish(Twist())
                    time.sleep(0.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
