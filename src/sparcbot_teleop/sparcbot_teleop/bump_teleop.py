"""Keyboard teleop that also records and plots how well the wheels follow the command.

    ros2 launch sparcbot_bringup robot.launch.py      # terminal 1
    ros2 run sparcbot_teleop record_teleop            # terminal 2

Run it with `ros2 run` (not from a launch file): it needs the terminal's keyboard.

Output (in --output-dir, default the current folder):
    teleop_<time>.csv   one row per control step
    teleop_<time>.png   the plot
"""

import argparse
import csv
import fcntl
import os
import signal
import sys
import termios
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sparcbot_interfaces.msg import WheelControllerDebug
from std_srvs.srv import Trigger

MIN_LINEAR_SPEED_M_S = 0.05
MAX_LINEAR_SPEED_M_S = 0.50
LINEAR_SPEED_STEP_M_S = 0.05

MIN_ANGULAR_SPEED_RAD_S = 0.25
MAX_ANGULAR_SPEED_RAD_S = 1.5
ANGULAR_SPEED_STEP_RAD_S = 0.25

PRINT_PERIOD_S = 0.2

CSV_COLUMNS = ('time_s', 'mode', 'cmd_linear_m_s', 'cmd_angular_rad_s',
               'left_command_m_s', 'left_measured_m_s', 'left_duty',
               'right_command_m_s', 'right_measured_m_s', 'right_duty')

MODE_NAMES = {
    WheelControllerDebug.MODE_VELOCITY: 'velocity',
    WheelControllerDebug.MODE_COAST: 'coast',
    WheelControllerDebug.MODE_BRAKE: 'brake',
}

HELP = """
Keyboard Teleop With Wheel PI + Kinematics  (recording)
=======================================================
  w : forward
  s : backward
  a : rotate left
  d : rotate right
  space : stop/coast
  b : brake
  + / = : increase linear speed
  -     : decrease linear speed
  ]     : increase angular speed
  [     : decrease angular speed
  r     : reset encoders/controllers
  q     : quit, save and plot the recording
"""

# Key -> label drawn on the plot when it is pressed.
EVENT_LABELS = {'w': 'fwd', 's': 'back', 'a': 'left', 'd': 'right', ' ': 'coast', 'b': 'brake'}


class RawTerminal:
    """Non-blocking, no-echo keyboard input; restores the terminal on exit."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        raw = termios.tcgetattr(self.fd)
        raw[3] &= ~(termios.ICANON | termios.ECHO)
        raw[6][termios.VMIN] = 0
        raw[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, raw)
        self.old_flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags | os.O_NONBLOCK)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.old_term)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags)

    def read_key(self):
        try:
            data = os.read(self.fd, 1)
        except BlockingIOError:
            return None
        return data.decode(errors='ignore') if data else None


class RecordTeleop(Node):

    def __init__(self):
        super().__init__('record_teleop')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.coast_client = self.create_client(Trigger, 'wheel_controller/coast')
        self.brake_client = self.create_client(Trigger, 'wheel_controller/brake')
        self.reset_controller_client = self.create_client(Trigger, 'wheel_controller/reset')
        self.reset_encoders_client = self.create_client(Trigger, 'sparcbot_driver/reset_encoders')

        # depth 100 so no control step is dropped while recording
        self.create_subscription(
            WheelControllerDebug, 'wheel_controller/debug', self.on_debug, 100)
        self.create_subscription(TwistStamped, 'body_velocity', self.on_body_velocity, 10)

        self.linear_speed_m_s = 0.10
        self.angular_speed_rad_s = 1.0

        self.linear_target_m_s = 0.0
        self.angular_target_rad_s = 0.0

        self.debug = None
        self.body_velocity = None

        # Recording
        self.rows = []          # one tuple per control step, see CSV_COLUMNS
        self.events = []        # (time_s, label) for key presses
        self._t0 = None

        self.create_timer(PRINT_PERIOD_S, self.print_status)

    # ------------------------------------------------------------------

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_debug(self, msg):
        self.debug = msg

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        self.rows.append((
            t - self._t0, MODE_NAMES.get(msg.mode, str(msg.mode)),
            self.linear_target_m_s, self.angular_target_rad_s,
            msg.left.target_m_s, msg.left.measured_m_s, msg.left.command,
            msg.right.target_m_s, msg.right.measured_m_s, msg.right.command))

    def on_body_velocity(self, msg):
        self.body_velocity = msg

    def mark(self, key):
        label = EVENT_LABELS.get(key.lower())
        if label is not None and self._t0 is not None:
            self.events.append((self._now_s() - self._t0, label))

    def call(self, client, name):
        if not client.service_is_ready():
            self.get_logger().warn(f'{name} service not available (is the robot launch running?)')
            return None
        return client.call_async(Trigger.Request())

    def drive(self, linear_m_s, angular_rad_s):
        self.linear_target_m_s = linear_m_s
        self.angular_target_rad_s = angular_rad_s
        msg = Twist()
        msg.linear.x = linear_m_s
        msg.angular.z = angular_rad_s
        self.cmd_vel_pub.publish(msg)

    def coast(self):
        self.linear_target_m_s = 0.0
        self.angular_target_rad_s = 0.0
        return self.call(self.coast_client, 'wheel_controller/coast')

    # ------------------------------------------------------------------

    def handle_key(self, key):
        """Return False when the user asked to quit."""
        self.mark(key)

        if key in ('w', 'W'):
            self.drive(+self.linear_speed_m_s, 0.0)
        elif key in ('s', 'S'):
            self.drive(-self.linear_speed_m_s, 0.0)
        elif key in ('a', 'A'):
            self.drive(0.0, +self.angular_speed_rad_s)
        elif key in ('d', 'D'):
            self.drive(0.0, -self.angular_speed_rad_s)
        elif key == ' ':
            self.coast()
        elif key in ('b', 'B'):
            self.linear_target_m_s = 0.0
            self.angular_target_rad_s = 0.0
            self.call(self.brake_client, 'wheel_controller/brake')
        elif key in ('+', '='):
            self.linear_speed_m_s = min(
                MAX_LINEAR_SPEED_M_S, self.linear_speed_m_s + LINEAR_SPEED_STEP_M_S)
            print(f'\nLinear speed = {self.linear_speed_m_s:g} m/s')
        elif key in ('-', '_'):
            self.linear_speed_m_s = max(
                MIN_LINEAR_SPEED_M_S, self.linear_speed_m_s - LINEAR_SPEED_STEP_M_S)
            print(f'\nLinear speed = {self.linear_speed_m_s:g} m/s')
        elif key == ']':
            self.angular_speed_rad_s = min(
                MAX_ANGULAR_SPEED_RAD_S, self.angular_speed_rad_s + ANGULAR_SPEED_STEP_RAD_S)
            print(f'\nAngular speed = {self.angular_speed_rad_s:g} rad/s')
        elif key == '[':
            self.angular_speed_rad_s = max(
                MIN_ANGULAR_SPEED_RAD_S, self.angular_speed_rad_s - ANGULAR_SPEED_STEP_RAD_S)
            print(f'\nAngular speed = {self.angular_speed_rad_s:g} rad/s')
        elif key in ('r', 'R'):
            self.call(self.reset_encoders_client, 'sparcbot_driver/reset_encoders')
            self.call(self.reset_controller_client, 'wheel_controller/reset')
            print('\nEncoders and controllers reset.')
        elif key in ('q', 'Q'):
            return False
        return True

    def print_status(self):
        if self.debug is None:
            return

        left = self.debug.left
        right = self.debug.right
        v = self.body_velocity.twist.linear.x if self.body_velocity else 0.0
        w = self.body_velocity.twist.angular.z if self.body_velocity else 0.0

        def f(x):
            return f'{x:7.4f}'

        print(
            f'CMD BODY: v={f(self.linear_target_m_s)} w={f(self.angular_target_rad_s)}'
            f' | TARGET WHEELS: L={f(left.target_m_s)} R={f(right.target_m_s)}'
            f' | MEAS WHEELS: L={f(left.measured_m_s)} R={f(right.measured_m_s)}'
            f' | ERROR: L={f(left.error_m_s)} R={f(right.error_m_s)}'
            f' | EST BODY: v={f(v)} w={f(w)}'
            f' | PWM: L={f(left.command)} R={f(right.command)}'
            f' | REC {len(self.rows)}'
        )


# ----------------------------------------------------------------------
# Results
# ----------------------------------------------------------------------

def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
        for row in rows:
            writer.writerow([f'{row[0]:.6f}', row[1]] + [f'{x:.6f}' for x in row[2:]])


def plot(path, rows, events, show=True):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    t = [r[0] for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    panels = (
        (axes[0], 'Left wheel', 4, 5),
        (axes[1], 'Right wheel', 7, 8),
    )
    for ax, title, cmd_i, meas_i in panels:
        ax.plot(t, [r[meas_i] for r in rows], label='measured', linewidth=1.2)
        ax.plot(t, [r[cmd_i] for r in rows], '--', label='commanded', linewidth=1.5)
        ax.set_title(title)
        ax.set_ylabel('wheel velocity [m/s]')

    for ax in axes:
        ax.axhline(0.0, color='black', linewidth=0.5, alpha=0.4)
        for t_event, _ in events:
            ax.axvline(t_event, color='grey', linewidth=0.7, alpha=0.5)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')

    # Key-press labels along the top panel.
    top = axes[0]
    for t_event, label in events:
        top.annotate(label, xy=(t_event, 1.0), xycoords=('data', 'axes fraction'),
                     xytext=(2, -2), textcoords='offset points', fontsize=8,
                     va='top', color='grey')

    axes[-1].set_xlabel('time [s]')
    fig.suptitle('Teleop recording: commanded vs measured wheel velocity')
    fig.tight_layout()
    fig.savefig(path, dpi=120)

    if show:
        plt.show()


def main(args=None):
    parser = argparse.ArgumentParser(description='Keyboard teleop that records and plots the run.')
    parser.add_argument('--output-dir', default='.', help='where to save the CSV and plot')
    parser.add_argument('--no-show', action='store_true', help='save the plot without opening it')
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])

    out_dir = Path(cli.output_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = out_dir / f'teleop_{datetime.now():%Y%m%d_%H%M%S}'

    # Handle Ctrl-C ourselves so ROS is still up to send the final coast.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RecordTeleop()
    print(HELP)

    # Ctrl-C only sets a flag; the loop then exits normally and the recording is saved.
    # (Raising KeyboardInterrupt inside ROS's C++ code can turn it into an unrelated
    # RuntimeError, which would skip saving.)
    stop_requested = False

    def on_sigint(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, on_sigint)

    try:
        with RawTerminal() as terminal:
            running = True
            while running and rclpy.ok() and not stop_requested:
                key = terminal.read_key()
                if key is not None:
                    running = node.handle_key(key)
                rclpy.spin_once(node, timeout_sec=0.01)
    except Exception as e:  # never lose the recording because of an error while driving
        print(f'\nERROR while driving: {e!r} - saving what was recorded.')
    finally:
        if rclpy.ok():
            future = node.coast()
            if future is not None:
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            print('\nStopped. Both motors are coasting.')

    try:
        rows = node.rows
        if not rows:
            print('Nothing recorded (is the robot launch running?).')
            return
        csv_path = stem.with_suffix('.csv')
        png_path = stem.with_suffix('.png')
        write_csv(csv_path, rows)
        print(f'Recorded {rows[-1][0]:.1f} s ({len(rows)} steps). Data saved to {csv_path}')
        plot(png_path, rows, node.events, show=not cli.no_show)
        print(f'Plot saved to {png_path}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()