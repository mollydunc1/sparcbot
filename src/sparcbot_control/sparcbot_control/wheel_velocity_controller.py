"""Wheel velocity controller: feedforward + PID.
"""

from dataclasses import dataclass

from sparcbot_control.feedforward import WheelFeedforward, WheelFeedforwardConfig


@dataclass
class PIDGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


@dataclass
class WheelVelocityControllerOutput:
    error_m_s: float = 0.0            # target - measured
    feedforward_command: float = 0.0  # from WheelFeedforward
    feedback_command: float = 0.0     # from the PID
    command: float = 0.0              # what is sent to the motor


class PID:
    """Scalar PID controller."""

    def __init__(self, gains: PIDGains):
        self.gains = gains

    def compute(self, error: float, dt_s: float) -> float:
        """Return the PID output for this time step.

        Args:
            error: target - measured.
            dt_s:  time since the previous call [s], > 0.
        """
        if dt_s <= 0.0:
            raise ValueError('PID.compute requires dt_s > 0')

        # TODO:
         # --- Proportional term ---
        p = self.gains.kp * error

        # --- Integral term ---
        self.integral += error * dt_s
        i = self.gains.ki * self.integral

        # --- Derivative term ---
        d = self.gains.kd * (error - self.prev_error) / dt_s
        self.prev_error = error

        # --- Total PID output ---
        return p + i + d

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0

class WheelVelocityController:
    """Feedforward + PID velocity controller for one wheel."""

    def __init__(self, feedforward_config: WheelFeedforwardConfig, gains: PIDGains):
        self.feedforward = WheelFeedforward(feedforward_config)
        self.pid = PID(gains)

    def compute(
        self,
        target_velocity_m_s: float,
        measured_velocity_m_s: float,
        dt_s: float,
    ) -> WheelVelocityControllerOutput:
        """Return the motor command (and its parts) for this time step."""
        u_ff = self.feedforward.compute(target_velocity_m_s)

        error = target_velocity_m_s - measured_velocity_m_s
        u_pid = self.pid.compute(error, dt_s)
        u = u_ff + u_pid
        return WheelVelocityControllerOutput(
            command=u,
            ff=u_ff,
            pid=u_pid,
        )
    def reset(self) -> None:
        """Reset the controller (the feedforward has no state)."""
        self.pid.reset()
