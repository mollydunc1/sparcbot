// odometry
//
// Thin ROS 2 wrapper around sparcbot_core's Odometry (encoder dead reckoning).
//
//   /wheel_states  -> Odometry::update() -> /odom  +  odom -> base_link TF
//
// The pose is integrated from the wheel *positions* in /wheel_states (exact
// encoder travel); the twist uses the filtered wheel velocities.
//
// Parameters (from sparcbot.yaml)
//   wheel_radius_m   wheel radius, converts wheel angle to distance   (shared /**)
//   base_width_m     wheel-to-wheel distance                          (shared /**)
//   odom_frame_id    parent frame, e.g. "odom"
//   base_frame_id    child frame, e.g. "base_link"
//   publish_tf       broadcast the odom -> base_link transform
//
// Topics
//   sub  wheel_states   sparcbot_interfaces/WheelStates
//   pub  odom           nav_msgs/Odometry
// TF
//   pub  odom -> base_link   (if publish_tf)
// Services
//   ~/reset             std_srvs/Trigger   set the pose back to the origin
//
// Note: if you zero the encoders with sparcbot_driver's ~/reset_encoders, call
// ~/reset here too. Zeroing the encoders makes the wheel positions jump, and
// ~/reset tells odometry to take the new readings as its starting point.

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "sparcbot/estimation/odometry.hpp"
#include "sparcbot/kinematics/wheel_geometry.hpp"
#include "sparcbot_interfaces/msg/wheel_states.hpp"
#include "sparcbot_interfaces/srv/set_pose.hpp"

// The core class is also called Odometry, so the ROS message gets an alias.
using OdometryMsg = nav_msgs::msg::Odometry;
using sparcbot_interfaces::msg::WheelStates;

namespace {

// Fixed uncertainty estimates for encoder-only odometry. Axes a planar robot
// cannot measure (z, roll, pitch) get a huge variance so a filter ignores them.
constexpr double XY_VARIANCE = 1e-3;
constexpr double YAW_VARIANCE = 1e-2;
constexpr double V_VARIANCE = 1e-3;
constexpr double W_VARIANCE = 1e-2;
constexpr double UNUSED_VARIANCE = 1e6;

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode()
        : Node("odometry")
    {
        WheelGeometryConfig geometry_config;
        geometry_config.radius_m = declare_parameter<double>("wheel_radius_m");
        geometry_ = std::make_unique<WheelGeometry>(geometry_config);

        DifferentialDriveKinematicsConfig kinematics_config;
        kinematics_config.base_width_m = declare_parameter<double>("base_width_m");
        odometry_ = std::make_unique<Odometry>(kinematics_config);

        odom_frame_id_ = declare_parameter<std::string>("odom_frame_id");
        base_frame_id_ = declare_parameter<std::string>("base_frame_id");
        publish_tf_ = declare_parameter<bool>("publish_tf");

        odom_pub_ = create_publisher<OdometryMsg>("odom", rclcpp::QoS(10));

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        wheel_states_sub_ = create_subscription<WheelStates>(
            "wheel_states", rclcpp::QoS(rclcpp::KeepLast(1)),
            [this](const WheelStates& msg) { onWheelStates(msg); });

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/reset",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                odometry_->reset();
                response->success = true;
                response->message = "Odometry reset to the origin.";
                RCLCPP_INFO(get_logger(), "Odometry reset to the origin.");
            });

        set_pose_srv_ = create_service<sparcbot_interfaces::srv::SetPose>(
        "~/set_pose",
        [this](
            const std::shared_ptr<sparcbot_interfaces::srv::SetPose::Request> request,
            std::shared_ptr<sparcbot_interfaces::srv::SetPose::Response> response)
        {
            if (!std::isfinite(request->x) ||
                !std::isfinite(request->y) ||
                !std::isfinite(request->yaw)) {
                response->success = false;
                response->message = "Pose values must be finite.";
                return;
            }

            Pose2D pose;
            pose.x_m = request->x;
            pose.y_m = request->y;
            pose.yaw_rad = request->yaw;

            odometry_->reset(pose);

            response->success = true;
            response->message = "Odometry pose corrected.";

            RCLCPP_INFO(
                get_logger(),
                "Pose correction: x=%.3f, y=%.3f, yaw=%.3f",
                pose.x_m, pose.y_m, pose.yaw_rad);
        });

        RCLCPP_INFO(get_logger(),
                    "odometry running (wheel radius %.4f m, base width %.4f m, %s -> %s, tf %s).",
                    geometry_config.radius_m,
                    kinematics_config.base_width_m,
                    odom_frame_id_.c_str(), base_frame_id_.c_str(),
                    publish_tf_ ? "on" : "off");
    }

private:
    void onWheelStates(const WheelStates& msg)
    {
        const double left_m = geometry_->angularToLinear(msg.left.position_rad);
        const double right_m = geometry_->angularToLinear(msg.right.position_rad);

        const OdometryState& state = odometry_->update(
            left_m, right_m, msg.left.velocity_m_s, msg.right.velocity_m_s);

        const Pose2D& pose = state.pose;

        // Yaw-only rotation as a quaternion.
        const double qz = std::sin(0.5 * pose.yaw_rad);
        const double qw = std::cos(0.5 * pose.yaw_rad);

        OdometryMsg odom;
        odom.header.stamp = msg.header.stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = base_frame_id_;

        odom.pose.pose.position.x = pose.x_m;
        odom.pose.pose.position.y = pose.y_m;
        odom.pose.pose.orientation.z = qz;
        odom.pose.pose.orientation.w = qw;

        odom.twist.twist.linear.x = state.twist.linear_m_s;
        odom.twist.twist.angular.z = state.twist.angular_rad_s;

        // 6x6 row-major covariance, order: x, y, z, roll, pitch, yaw.
        odom.pose.covariance[0] = XY_VARIANCE;
        odom.pose.covariance[7] = XY_VARIANCE;
        odom.pose.covariance[14] = UNUSED_VARIANCE;
        odom.pose.covariance[21] = UNUSED_VARIANCE;
        odom.pose.covariance[28] = UNUSED_VARIANCE;
        odom.pose.covariance[35] = YAW_VARIANCE;

        odom.twist.covariance[0] = V_VARIANCE;
        odom.twist.covariance[7] = UNUSED_VARIANCE;
        odom.twist.covariance[14] = UNUSED_VARIANCE;
        odom.twist.covariance[21] = UNUSED_VARIANCE;
        odom.twist.covariance[28] = UNUSED_VARIANCE;
        odom.twist.covariance[35] = W_VARIANCE;

        odom_pub_->publish(odom);

        if (tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = odom.header;
            tf.child_frame_id = base_frame_id_;
            tf.transform.translation.x = pose.x_m;
            tf.transform.translation.y = pose.y_m;
            tf.transform.rotation.z = qz;
            tf.transform.rotation.w = qw;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    std::unique_ptr<WheelGeometry> geometry_;
    std::unique_ptr<Odometry> odometry_;

    std::string odom_frame_id_;
    std::string base_frame_id_;
    bool publish_tf_ = true;

    rclcpp::Publisher<OdometryMsg>::SharedPtr odom_pub_;
    rclcpp::Subscription<WheelStates>::SharedPtr wheel_states_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Service<sparcbot_interfaces::srv::SetPose>::SharedPtr set_pose_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<OdometryNode>());
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("odometry"), "%s", e.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
