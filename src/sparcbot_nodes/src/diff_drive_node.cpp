// diff_drive
//
// Thin ROS 2 wrapper around sparcbot_core's DifferentialDriveKinematics.
//
//   /cmd_vel (body v, w)        -> inverse kinematics -> /wheel_targets (left, right m/s)
//   /wheel_states (measured)    -> forward kinematics -> /body_velocity (measured v, w)
//
// Encoder odometry (/odom + odom->base_link TF) lives in odometry_node.
//
// Topics
//   sub  cmd_vel         geometry_msgs/Twist
//   pub  wheel_targets   sparcbot_interfaces/WheelVelocities
//   sub  wheel_states    sparcbot_interfaces/WheelStates
//   pub  body_velocity   geometry_msgs/TwistStamped

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sparcbot/kinematics/differential_drive_kinematics.hpp"
#include "sparcbot_interfaces/msg/wheel_states.hpp"
#include "sparcbot_interfaces/msg/wheel_velocities.hpp"

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using sparcbot_interfaces::msg::WheelStates;
using WheelVelocitiesMsg = sparcbot_interfaces::msg::WheelVelocities;

namespace {

class DiffDriveNode : public rclcpp::Node {
public:
    DiffDriveNode()
        : Node("diff_drive")
    {
        DifferentialDriveKinematicsConfig config;
        config.base_width_m = declare_parameter<double>("base_width_m");
        kinematics_ = std::make_unique<DifferentialDriveKinematics>(config);

        const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1));

        targets_pub_ = create_publisher<WheelVelocitiesMsg>("wheel_targets", rclcpp::QoS(10));
        body_velocity_pub_ = create_publisher<TwistStamped>("body_velocity", control_qos);

        cmd_vel_sub_ = create_subscription<Twist>(
            "cmd_vel", rclcpp::QoS(10),
            [this](const Twist& msg) { onCmdVel(msg); });

        wheel_states_sub_ = create_subscription<WheelStates>(
            "wheel_states", control_qos,
            [this](const WheelStates& msg) { onWheelStates(msg); });

        RCLCPP_INFO(get_logger(), "diff_drive running (base width %.5f m).", config.base_width_m);
    }

private:
    void onCmdVel(const Twist& msg)
    {
        const WheelVelocities targets = kinematics_->robotToWheelVelocities(msg.linear.x, msg.angular.z);

        WheelVelocitiesMsg out;
        out.header.stamp = get_clock()->now();
        out.header.frame_id = "base_link";
        out.left_m_s = targets.left_m_s;
        out.right_m_s = targets.right_m_s;
        targets_pub_->publish(out);
    }

    void onWheelStates(const WheelStates& msg)
    {
        const RobotVelocity v =
            kinematics_->wheelToRobotVelocity(msg.left.velocity_m_s, msg.right.velocity_m_s);

        TwistStamped out;
        out.header = msg.header;
        out.twist.linear.x = v.linear_m_s;
        out.twist.angular.z = v.angular_rad_s;
        body_velocity_pub_->publish(out);
    }

    std::unique_ptr<DifferentialDriveKinematics> kinematics_;

    rclcpp::Publisher<WheelVelocitiesMsg>::SharedPtr targets_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr body_velocity_pub_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<WheelStates>::SharedPtr wheel_states_sub_;
};

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<DiffDriveNode>());
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("diff_drive"), "%s", e.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
