"""wheel_controller node: runs the WheelController for both wheels.

Given ROS plumbing; the controller itself is in wheel_controller.py.

Every /wheel_states message (100 Hz, from sparcbot_driver):
  * VELOCITY mode: compute left/right commands from the latest /cmd_vel and the
    measured wheel velocities, and publish them on /wheel_cmd as duty.
  * COAST / BRAKE mode: publish a coast / brake command.

Parameter `log_file` (sparcbot.yaml): file that WheelController.log() writes to
('' = no logging).

Modes:
  * starts in COAST
  * any /cmd_vel -> VELOCITY (holds the last command)
  * ~/coast, ~/brake  -> zero the command, reset the controller, coast / brake
  * ~/reset           -> reset the controller (mode unchanged)

Topics
  sub  cmd_vel                 geometry_msgs/Twist
  sub  wheel_states            sparcbot_interfaces/WheelStates
  pub  wheel_cmd               sparcbot_interfaces/WheelCommand
  pub  ~/debug                 sparcbot_interfaces/WheelControllerDebug
Services
  ~/coast  ~/brake  ~/reset    std_srvs/Trigger
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from sparcbot_interfaces.msg import (
    WheelCommand,
    WheelControllerDebug,
    WheelControllerOutput as OutputMsg,
    WheelStates,
)
from std_srvs.srv import Trigger

from sparcbot_control.controller_config import WheelControllerConfig
from sparcbot_control.wheel_controller import (
    LEFT,
    RIGHT,
    WheelController,
    WheelControllerOutput,
)

FEEDFORWARD_KEYS = (
    'forward_static',
    'forward_velocity_gain',
    'reverse_static',
    'reverse_velocity_gain',
    'forward_breakaway',
    'reverse_breakaway',
    'stopped_velocity_threshold',
    'max_command',
    'zero_velocity_threshold',
)

MODE_VELOCITY = WheelControllerDebug.MODE_VELOCITY
MODE_COAST = WheelControllerDebug.MODE_COAST
MODE_BRAKE = WheelControllerDebug.MODE_BRAKE


class WheelControllerNode(Node):

    def __init__(self):
        super().__init__('wheel_controller')

        log_file = self._required_string('log_file')
        self.controller = WheelController(
            self._controller_config('left'),
            self._controller_config('right'),
            self._required_double('base_width_m'),
            log_file)

        self.mode = MODE_COAST
        self.linear_m_s = 0.0          # latest /cmd_vel
        self.angular_rad_s = 0.0

        control_qos = QoSProfile(depth=1)

        self.cmd_pub = self.create_publisher(WheelCommand, 'wheel_cmd', control_qos)
        self.debug_pub = self.create_publisher(WheelControllerDebug, '~/debug', 10)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(WheelStates, 'wheel_states', self.on_wheel_states, control_qos)

        self.create_service(Trigger, '~/coast', self.on_coast)
        self.create_service(Trigger, '~/brake', self.on_brake)
        self.create_service(Trigger, '~/reset', self.on_reset)

        self._not_implemented_reported = False

        self.get_logger().info('wheel_controller ready (COAST until the first /cmd_vel).')
        if log_file:
            self.get_logger().info(f'WheelController.log() writes to {log_file}')

    # ------------------------------------------------------------------
    # Parameters (all required: they come from sparcbot.yaml)
    # ------------------------------------------------------------------

    def _required(self, name: str, kind):
        self.declare_parameter(name, kind)
        value = self.get_parameter(name).value
        if value is None:
            raise RuntimeError(f"Missing parameter '{name}' (check config/sparcbot.yaml)")
        return value

    def _required_double(self, name: str) -> float:
        return float(self._required(name, Parameter.Type.DOUBLE))

    def _required_string(self, name: str) -> str:
        return str(self._required(name, Parameter.Type.STRING))

    def _controller_config(self, wheel: str) -> WheelControllerConfig:
        values = {key: self._required_double(f'{wheel}.{key}') for key in ('kp', 'ki', 'kd')}
        values.update({
            key: self._required_double(f'{wheel}.feedforward.{key}')
            for key in FEEDFORWARD_KEYS
        })
        return WheelControllerConfig(**values)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_cmd_vel(self, msg: Twist) -> None:
        self.linear_m_s = msg.linear.x
        self.angular_rad_s = msg.angular.z
        self.mode = MODE_VELOCITY

    def on_wheel_states(self, msg: WheelStates) -> None:
        dt_s = msg.dt_s
        measured = [msg.left.velocity_m_s, msg.right.velocity_m_s]
        targets = [0.0, 0.0]
        outputs = [WheelControllerOutput(), WheelControllerOutput()]

        cmd = WheelCommand()
        cmd.header.stamp = msg.header.stamp

        if self.mode == MODE_VELOCITY:
            try:
                targets = self.controller.diff_drive_kinematics(
                    self.linear_m_s, self.angular_rad_s)
                outputs = self.controller.compute(
                    self.linear_m_s, self.angular_rad_s, measured, dt_s)
            except NotImplementedError as e:
                if not self._not_implemented_reported:
                    self.get_logger().error(f'{e} - motors will coast.')
                    self._not_implemented_reported = True
                cmd.mode = WheelCommand.MODE_COAST
                self.cmd_pub.publish(cmd)
                return

            cmd.mode = WheelCommand.MODE_DUTY
            cmd.left = float(outputs[LEFT].command)
            cmd.right = float(outputs[RIGHT].command)

        elif self.mode == MODE_BRAKE:
            cmd.mode = WheelCommand.MODE_BRAKE

        else:
            cmd.mode = WheelCommand.MODE_COAST

        self.cmd_pub.publish(cmd)
        self._publish_debug(msg, targets, measured, outputs)

    def _publish_debug(self, states, targets, measured, outputs) -> None:
        debug = WheelControllerDebug()
        debug.header.stamp = states.header.stamp
        debug.mode = self.mode
        debug.dt_s = states.dt_s
        debug.left = self._output_msg(targets[LEFT], measured[LEFT], outputs[LEFT])
        debug.right = self._output_msg(targets[RIGHT], measured[RIGHT], outputs[RIGHT])
        self.debug_pub.publish(debug)

    @staticmethod
    def _output_msg(target, measured, output) -> OutputMsg:
        msg = OutputMsg()
        msg.target_m_s = float(target)
        msg.measured_m_s = float(measured)
        msg.error_m_s = float(output.error_m_s)
        msg.feedforward_command = float(output.feedforward_command)
        msg.feedback_command = float(output.feedback_command)
        msg.command = float(output.command)
        return msg

    def _stop(self, mode: int) -> None:
        self.mode = mode
        self.linear_m_s = 0.0
        self.angular_rad_s = 0.0
        self.controller.reset()

    def on_coast(self, request, response):
        self._stop(MODE_COAST)
        response.success = True
        response.message = 'Coasting.'
        return response

    def on_brake(self, request, response):
        self._stop(MODE_BRAKE)
        response.success = True
        response.message = 'Braking.'
        return response

    def on_reset(self, request, response):
        self.controller.reset()
        response.success = True
        response.message = 'Controller reset.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.controller.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()