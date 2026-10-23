"""Velocity step test: check how well the wheel controller tracks a command.

    ros2 run sparcbot_control velocity_step_test --speeds 0.1 0.2 0.3 --duration 5
    ros2 run sparcbot_control velocity_step_test --no-show      # save the plot only

Output (in --output-dir, default the current folder):
    velocity_step_<time>.csv   one row per control step
    velocity_step_<time>.png   the plot
"""


import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path
 
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sparcbot_interfaces.msg import WheelControllerDebug
from std_srvs.srv import Trigger
 
PUBLISH_PERIOD_S = 0.05   # re-send /cmd_vel at 20 Hz
CSV_COLUMNS = ('time_s', 'cmd_linear_m_s',
               'left_command_m_s', 'left_measured_m_s',
               'right_command_m_s', 'right_measured_m_s')
 
 
class VelocityStepTest(Node):
 
    def __init__(self):
        super().__init__('velocity_step_test')
 
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            WheelControllerDebug, 'wheel_controller/debug', self.on_debug, 100)
 
        self.coast_client = self.create_client(Trigger, 'wheel_controller/coast')
        self.reset_client = self.create_client(Trigger, 'wheel_controller/reset')
        self.params_client = self.create_client(GetParameters, 'wheel_controller/get_parameters')
 
        self.recording = False
        self.linear_m_s = 0.0
        self.rows = []
        self._t0 = None
 
    def on_debug(self, msg):
        if not self.recording or msg.mode != WheelControllerDebug.MODE_VELOCITY:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        self.rows.append((t - self._t0, self.linear_m_s,
                          msg.left.target_m_s, msg.left.measured_m_s,
                          msg.right.target_m_s, msg.right.measured_m_s))
 
    # ------------------------------------------------------------------
 
    def wait_for_controller(self, timeout_s=5.0):
        for client in (self.coast_client, self.reset_client):
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise RuntimeError(
                    f'{client.srv_name} not available. Start the robot first:\n'
                    '    ros2 launch sparcbot_bringup robot.launch.py   (with mode: velocity)')
 
    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result() if future.done() else None
 
    def gains_text(self):
        """'L kp=.. ki=.. kd=.. | R ...' for the plot title, or '' if unavailable."""
        if not self.params_client.wait_for_service(timeout_sec=1.0):
            return ''
        names = [f'{w}.{k}' for w in ('left', 'right') for k in ('kp', 'ki', 'kd')]
        result = self.call(self.params_client, GetParameters.Request(names=names))
        if result is None:
            return ''
        v = [p.double_value for p in result.values]
        return (f'L kp={v[0]:g} ki={v[1]:g} kd={v[2]:g}   '
                f'R kp={v[3]:g} ki={v[4]:g} kd={v[5]:g}')
 
    def send(self, linear_m_s):
        msg = Twist()
        msg.linear.x = linear_m_s
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
 
    def stop(self):
        self.recording = False
        self.linear_m_s = 0.0
        self.send(0.0)
        self.call(self.coast_client, Trigger.Request())
 
    def run(self, speeds, duration_s):
        self.call(self.coast_client, Trigger.Request())
        self.spin_for(0.5)
        self.call(self.reset_client, Trigger.Request())
 
        self.rows = []
        self._t0 = None
        self.recording = True
 
        for speed in speeds:
            print(f'  {speed:.2f} m/s for {duration_s:g} s ...')
            self.linear_m_s = speed
            end = time.monotonic() + duration_s
            next_send = 0.0
            while time.monotonic() < end:
                if time.monotonic() >= next_send:
                    self.send(speed)
                    next_send = time.monotonic() + PUBLISH_PERIOD_S
                rclpy.spin_once(self, timeout_sec=0.005)
 
        self.stop()
        return self.rows
 
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)
 
 
# ----------------------------------------------------------------------
# Results
# ----------------------------------------------------------------------
 
def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
        for row in rows:
            writer.writerow([f'{x:.6f}' for x in row])
 
 
def summary(rows, speeds):
    """Per speed and wheel: mean measured velocity over the second half of the step."""
    lines = []
    for speed in speeds:
        segment = [r for r in rows if r[1] == speed]
        if not segment:
            continue
        steady = segment[len(segment) // 2:]
        for name, cmd_i, meas_i in (('left', 2, 3), ('right', 4, 5)):
            cmd = sum(r[cmd_i] for r in steady) / len(steady)
            meas = sum(r[meas_i] for r in steady) / len(steady)
            err = cmd - meas
            pct = 100.0 * err / cmd if cmd else float('nan')
            lines.append(f'  {speed:5.2f} m/s  {name:5}  command {cmd:6.3f}  '
                         f'measured {meas:6.3f}  error {err:+.3f} ({pct:+.1f}%)')
    return '\n'.join(lines)
 
 
def plot(path, rows, speeds, duration_s, title_suffix='', show=True, band=True,
         settling_time_s=1.0):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
 
    t = [r[0] for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
 
    for ax, (name, cmd_i, meas_i) in zip(axes, (('Left', 2, 3), ('Right', 4, 5))):
        ax.plot(t, [r[meas_i] for r in rows], label='measured', linewidth=1.2)
        cmd = [r[cmd_i] for r in rows]
        ax.plot(t, cmd, '--', label='commanded', linewidth=1.5)
        if band:
            # Band half-width for each step: commanded velocity / step duration.
            half = [abs(c) / duration_s for c in cmd]
            upper = [c + h for c, h in zip(cmd, half)]
            lower = [c - h for c, h in zip(cmd, half)]
            ax.plot(t, upper, ':', color='grey', linewidth=1.0,
                    label=f'± velocity / {duration_s:g} s')
            ax.plot(t, lower, ':', color='grey', linewidth=1.0)
            ax.fill_between(t, lower, upper, color='grey', alpha=0.12, linewidth=0)
        for k in range(1, len(speeds)):
            ax.axvline(k * duration_s, color='grey', linewidth=0.6, alpha=0.5)
        if settling_time_s > 0:
            # Settling-time target: x seconds after the start of each step.
            for k in range(len(speeds)):
                ax.axvline(k * duration_s + settling_time_s, color='tab:red',
                           linestyle='-.', linewidth=1.0,
                           label=f'settling target ({settling_time_s:g} s)' if k == 0 else None)
        ax.set_title(f'{name} wheel')
        ax.set_ylabel('wheel velocity [m/s]')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper left')
 
    axes[-1].set_xlabel('time [s]')
    title = 'Velocity step test: ' + ', '.join(f'{s:g}' for s in speeds) + ' m/s'
    if title_suffix:
        title += f'\n{title_suffix}'
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
 
    if show:
        plt.show()
 
 
def main(args=None):
    parser = argparse.ArgumentParser(description='Wheel velocity step test.')
    parser.add_argument('--speeds', type=float, nargs='+', default=[0.25, -0.5, 0.75],
                        help='linear velocities to test [m/s] (default 0.2 0.6 1.0)')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='seconds at each speed (default 3)')
    parser.add_argument('--settling-time', type=float, default=0.5,
                        help='draw a vertical line this many seconds after each step starts '
                             '(default 1.0, 0 = none)')
    parser.add_argument('--no-band', action='store_true',
                        help='do not draw the +/- (velocity / duration) band around the command')
    parser.add_argument('--output-dir', default='.', help='where to save the CSV and plot')
    parser.add_argument('--no-show', action='store_true', help='save the plot without opening it')
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])
 
    out_dir = Path(cli.output_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = out_dir / f'velocity_step_{datetime.now():%Y%m%d_%H%M%S}'
 
    # Handle Ctrl-C ourselves so ROS is still up to stop the robot.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VelocityStepTest()
    rows = []
 
    try:
        node.wait_for_controller()
        gains = node.gains_text()
        total = len(cli.speeds) * cli.duration
        print(f'\nVelocity step test: {cli.speeds} m/s, {cli.duration:g} s each ({total:g} s).')
        print('Make sure the robot has room (or is on a stand). Ctrl-C stops it.')
        input('Press ENTER to start...')
        rows = node.run(cli.speeds, cli.duration)
 
    except (KeyboardInterrupt, EOFError):
        print('\nStopped early.')
        rows = node.rows
        gains = locals().get('gains', '')
    except RuntimeError as e:
        print(f'\nERROR: {e}')
        gains = ''
    finally:
        try:
            node.stop()
        except Exception:
            pass
 
    try:
        if not rows:
            print('No controller data recorded (is wheel_controller running in velocity mode?).')
            return
 
        csv_path = stem.with_suffix('.csv')
        png_path = stem.with_suffix('.png')
        write_csv(csv_path, rows)
        print(f'\nSteady state (second half of each step):\n{summary(rows, cli.speeds)}')
        print(f'\nData saved to {csv_path}')
        plot(png_path, rows, cli.speeds, cli.duration, gains, show=not cli.no_show,
             band=not cli.no_band, settling_time_s=cli.settling_time)
        print(f'Plot saved to {png_path}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 