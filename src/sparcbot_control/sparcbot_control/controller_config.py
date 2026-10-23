"""Constants for one wheel's controller.

Do not put your values here: set them in sparcbot_bringup/config/sparcbot.yaml
under wheel_controller -> left / right. The node reads that file and builds one
WheelControllerConfig per wheel; the field names below match the YAML keys.
(The defaults here are only used by the tests.)

In wheel_controller.py you read them as self.config[LEFT].kp,
self.config[RIGHT].forward_static, and so on.
"""

from dataclasses import dataclass


@dataclass
class WheelControllerConfig:
    # PID gains                                   YAML: <wheel>.kp / ki / kd
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0

    # Feedforward line: duty = static + velocity_gain * |v|
    #                                             YAML: <wheel>.feedforward.*
    forward_static: float = 0.0
    forward_velocity_gain: float = 0.0
    reverse_static: float = 0.0
    reverse_velocity_gain: float = 0.0

    # Deadband: minimum duty to start a stopped wheel
    forward_breakaway: float = 0.0
    reverse_breakaway: float = 0.0
    stopped_velocity_threshold: float = 0.005   # |v| below this counts as stopped [m/s]

    # Limits
    max_command: float = 1.0                    # feedforward is limited to +/- this
    zero_velocity_threshold: float = 1e-4       # |target| below this gives 0 feedforward [m/s]

    def __post_init__(self):
        non_negative = (
            'forward_static', 'forward_velocity_gain',
            'reverse_static', 'reverse_velocity_gain',
            'forward_breakaway', 'reverse_breakaway',
            'stopped_velocity_threshold', 'zero_velocity_threshold',
        )
        for name in non_negative:
            if getattr(self, name) < 0.0:
                raise ValueError(f'{name} must be >= 0')
        if self.max_command <= 0.0 or self.max_command > 1.0:
            raise ValueError('max_command must be in (0, 1]')
