#include "sparcbot/estimation/odometry.hpp"

#include <cmath>

namespace {

// Wrap an angle to [-pi, pi].
double wrapAngle(double angle_rad, double two_pi)
{
    return angle_rad - two_pi * std::floor((angle_rad + 0.5 * two_pi) / two_pi);
}

// sin(h) / h, with a series expansion near zero to avoid 0/0.
double sinc(double h)
{
    if (std::abs(h) < 1e-6) {
        return 1.0 - (h * h) / 6.0;
    }
    return std::sin(h) / h;
}

} // namespace

Odometry::Odometry(const DifferentialDriveKinematicsConfig& config)
    : kinematics_(config)
{
}

void Odometry::reset(const Pose2D& pose)
{
    state_ = OdometryState{};
    state_.pose = pose;
    state_.pose.yaw_rad = wrapAngle(pose.yaw_rad, TWO_PI);
    initialized_ = false;
}

const OdometryState& Odometry::update(
    double left_position_m,
    double right_position_m,
    double left_velocity_m_s,
    double right_velocity_m_s)
{
    // Twist from the filtered wheel velocities (same math as /body_velocity).
    state_.twist = kinematics_.wheelToRobotVelocity(left_velocity_m_s, right_velocity_m_s);

    if (!initialized_) {
        previous_left_m_ = left_position_m;
        previous_right_m_ = right_position_m;
        initialized_ = true;
        return state_;
    }

    const double delta_left_m = left_position_m - previous_left_m_;
    const double delta_right_m = right_position_m - previous_right_m_;
    previous_left_m_ = left_position_m;
    previous_right_m_ = right_position_m;

    // The wheel -> body map is linear, so it works on distances exactly as it
    // does on velocities:
    //
    //   ds     = (dR + dL) / 2        distance travelled by the robot centre
    //   dtheta = (dR - dL) / width    heading change
    //
    const RobotVelocity step = kinematics_.wheelToRobotVelocity(delta_left_m, delta_right_m);
    const double ds_m = step.linear_m_s;
    const double dyaw_rad = step.angular_rad_s;

    // Exact integration along a circular arc. The robot moves along the chord
    // of the arc, which points at the mid-heading, and the chord is shorter
    // than the arc by a factor sinc(dtheta / 2):
    //
    //   x += ds * sinc(dtheta/2) * cos(theta + dtheta/2)
    //   y += ds * sinc(dtheta/2) * sin(theta + dtheta/2)
    //
    // This reduces to a straight line for dtheta -> 0 and stays accurate even
    // if a control period is late and the step is large.
    const double mid_yaw_rad = state_.pose.yaw_rad + 0.5 * dyaw_rad;
    const double chord_m = ds_m * sinc(0.5 * dyaw_rad);

    state_.pose.x_m += chord_m * std::cos(mid_yaw_rad);
    state_.pose.y_m += chord_m * std::sin(mid_yaw_rad);
    state_.pose.yaw_rad = wrapAngle(state_.pose.yaw_rad + dyaw_rad, TWO_PI);

    return state_;
}
