#pragma once

#include "sparcbot/kinematics/differential_drive_kinematics.hpp"

// Planar pose of the robot (base_link) in the odometry frame (odom).
//
//   x, y   position of the robot centre [m]
//   yaw    heading, counter-clockwise positive, wrapped to [-pi, pi] [rad]
struct Pose2D {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
};

struct OdometryState {
    // Pose of base_link in the odom frame.
    Pose2D pose;

    // Body velocity (v, omega) in the base_link frame.
    RobotVelocity twist;
};

// Encoder-only differential-drive odometry (dead reckoning).
//
// The pose is integrated from the change in wheel *position* between two
// updates, not from velocity * dt. Encoder positions are exact counts, so the
// integrated distance has no filter lag and does not depend on dt jitter.
//
// The twist is built from the (filtered) wheel velocities you pass in, because
// differentiating raw encoder counts over one control period is very coarse.
//
// Pure C++, no ROS and no hardware: it can be unit-tested on a laptop.
class Odometry {
public:
    explicit Odometry(const DifferentialDriveKinematicsConfig& config);

    // Set the pose and forget the encoder baseline. The next update() latches
    // the wheel positions it receives as the new baseline (no motion is
    // integrated on that call). Call this whenever the encoders are zeroed.
    void reset(const Pose2D& pose = Pose2D{});

    // Update from absolute wheel travel (distance rolled since any fixed
    // reference, positive = forward) and the current wheel rim velocities.
    const OdometryState& update(
        double left_position_m,
        double right_position_m,
        double left_velocity_m_s,
        double right_velocity_m_s);

    const OdometryState& state() const { return state_; }
    const Pose2D& pose() const { return state_.pose; }

private:
    DifferentialDriveKinematics kinematics_;
    OdometryState state_;

    double previous_left_m_ = 0.0;
    double previous_right_m_ = 0.0;
    bool initialized_ = false;

    static constexpr double PI = 3.14159265358979323846264338327950288;
    static constexpr double TWO_PI = 2.0 * PI;
};
