#include "sparcbot/config/robot_config.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#ifndef SPARCBOT_DEFAULT_CONFIG
#error "SPARCBOT_DEFAULT_CONFIG must be defined by the build (see sparcbot_core/CMakeLists.txt)"
#endif

namespace sparcbot::config {

namespace {

const YAML::Node& root()
{
    static const YAML::Node node = [] {
        const std::string path = configPath();
        try {
            return YAML::LoadFile(path);
        }
        catch (const YAML::Exception& e) {
            throw std::runtime_error("Could not load SPARCBot config '" + path + "': " + e.what());
        }
    }();
    return node;
}

// Returns <node_name>.ros__parameters[.<section>]
YAML::Node section(const std::string& node_name, const std::string& sub = "")
{
    const YAML::Node& r = root();
    const YAML::Node params = r[node_name]["ros__parameters"];

    if (!params) {
        throw std::runtime_error(
            "Missing '" + node_name + ": ros__parameters:' in " + configPath());
    }

    if (sub.empty()) {
        return params;
    }

    const YAML::Node s = params[sub];
    if (!s) {
        throw std::runtime_error(
            "Missing '" + node_name + "." + sub + "' in " + configPath());
    }
    return s;
}

template <typename T>
T get(const YAML::Node& node, const std::string& key, const std::string& where)
{
    const YAML::Node value = node[key];
    if (!value) {
        throw std::runtime_error(
            "Missing parameter '" + where + "." + key + "' in " + configPath());
    }
    try {
        return value.as<T>();
    }
    catch (const YAML::Exception&) {
        throw std::runtime_error(
            "Parameter '" + where + "." + key + "' in " + configPath() + " has the wrong type");
    }
}

MotorConfig motor(const std::string& key)
{
    const std::string where = "sparcbot_driver." + key;
    const YAML::Node driver = section("sparcbot_driver");
    const YAML::Node m = section("sparcbot_driver", key);

    MotorConfig config;
    config.name = get<std::string>(m, "name", where);
    config.gpio_dir = get<int>(m, "gpio_dir", where);
    config.gpio_ps = get<int>(m, "gpio_ps", where);
    config.gpio_enc_a = get<int>(m, "gpio_enc_a", where);
    config.gpio_enc_b = get<int>(m, "gpio_enc_b", where);
    config.pwm_chip = get<int>(m, "pwm_chip", where);
    config.pwm_channel = get<int>(m, "pwm_channel", where);
    config.pwm_frequency_hz = get<int>(m, "pwm_frequency_hz", where);
    config.counts_per_output_rev = get<double>(driver, "counts_per_rev", "sparcbot_driver");
    config.motor_sign = get<int>(m, "motor_sign", where);
    config.encoder_sign = get<int>(m, "encoder_sign", where);
    return config;
}

WheelFeedforwardConfig feedforward(const std::string& wheel)
{
    const std::string where = "wheel_controller." + wheel + ".feedforward";
    const YAML::Node w = section("wheel_controller", wheel);
    const YAML::Node f = w["feedforward"];
    if (!f) {
        throw std::runtime_error("Missing '" + where + "' in " + configPath());
    }

    WheelFeedforwardConfig config;
    config.forward_static = get<double>(f, "forward_static", where);
    config.forward_velocity_gain = get<double>(f, "forward_velocity_gain", where);
    config.reverse_static = get<double>(f, "reverse_static", where);
    config.reverse_velocity_gain = get<double>(f, "reverse_velocity_gain", where);
    config.forward_breakaway = get<double>(f, "forward_breakaway", where);
    config.reverse_breakaway = get<double>(f, "reverse_breakaway", where);
    config.stopped_velocity_threshold = get<double>(f, "stopped_velocity_threshold", where);
    config.max_command = get<double>(f, "max_command", where);
    config.zero_velocity_threshold = get<double>(f, "zero_velocity_threshold", where);
    return config;
}

} // namespace

std::string configPath()
{
    if (const char* env = std::getenv("SPARCBOT_CONFIG"); env != nullptr && *env != '\0') {
        return env;
    }
    return SPARCBOT_DEFAULT_CONFIG;
}

MotorConfig leftMotor()
{
    return motor("left_motor");
}

MotorConfig rightMotor()
{
    return motor("right_motor");
}

WheelGeometryConfig wheelGeometry()
{
    WheelGeometryConfig config;
    config.radius_m = get<double>(section("/**"), "wheel_radius_m", "/**");
    return config;
}

WheelStateEstimatorConfig wheelStateEstimator()
{
    const YAML::Node driver = section("sparcbot_driver");

    WheelStateEstimatorConfig config;
    config.counts_per_rev = get<double>(driver, "counts_per_rev", "sparcbot_driver");
    config.velocity_cutoff_hz = get<double>(driver, "velocity_cutoff_hz", "sparcbot_driver");
    return config;
}

DifferentialDriveKinematicsConfig differentialDriveKinematics()
{
    DifferentialDriveKinematicsConfig config;
    config.base_width_m = get<double>(section("/**"), "base_width_m", "/**");
    return config;
}

WheelFeedforwardConfig leftWheelFeedforward()
{
    return feedforward("left");
}

WheelFeedforwardConfig rightWheelFeedforward()
{
    return feedforward("right");
}

} // namespace sparcbot::config
