#pragma once
#include "rclcpp/rclcpp.hpp"

/* Utils */
#include "candle_ros2/utils/candle_params.hpp"

/* CANdle-SDK */
#include "candle.hpp"

inline candleParams_S readParams(const rclcpp::Node::SharedPtr& node)
{
    node->declare_parameter<std::string>("data_rate", "1M");
    node->declare_parameter<std::string>("bus", "USB");
    node->declare_parameter<std::string>("default_qos", "Reliable");
    node->declare_parameter<std::string>("joint_name_prefix", "md_");
    node->declare_parameter<double>("gripper_open_position_rad", 0.0);
    node->declare_parameter<double>("gripper_closed_position_rad", 0.62);
    node->declare_parameter<double>("gripper_impedance_kp", 12.5);
    node->declare_parameter<double>("gripper_impedance_kd", 0.05);
    node->declare_parameter<double>("gripper_velocity_limit_rad_s", 6.0);
    node->declare_parameter<double>("gripper_torque_limit_nm", 3.0);
    node->declare_parameter<double>("soft_close_fast_torque_limit_nm", 4.0);
    node->declare_parameter<double>("soft_close_slow_torque_limit_nm", 4.0);
    node->declare_parameter<double>("soft_close_profile_acceleration_rad_s2", 100.0);
    node->declare_parameter<double>("soft_close_profile_deceleration_rad_s2", 100.0);
    node->declare_parameter<double>("soft_close_closed_tol_rad", 0.005);
    node->declare_parameter<int>("soft_close_fast_duration_ms", 300);
    node->declare_parameter<int>("health_publish_period_ms", 200);
    node->declare_parameter<double>("gripper_state_position_tolerance_rad", 0.01);
    node->declare_parameter<double>("gripper_state_moving_velocity_rad_s", 0.12);
    node->declare_parameter<double>("encoder_wrap_period_rad", 0.62831853);
    node->declare_parameter<double>("position_recovery_max_delta_rad", 0.25);
    node->declare_parameter<int>("position_recovery_samples", 3);
    node->declare_parameter<int>("position_recovery_retry_ms", 25);
    node->declare_parameter<std::string>("startup_position_policy", "restore_or_home");
    node->declare_parameter<std::string>(
        "position_state_file", "~/.ros/candle_ros2_position_state.json");
    node->declare_parameter<int>("position_state_write_period_ms", 1000);
    node->declare_parameter<double>("position_state_min_change_rad", 0.005);
    node->declare_parameter<std::string>(
        "homing_direction_by_id", "342:-1,343:-1,345:-1");
    node->declare_parameter<double>("homing_torque_nm", 0.3);
    node->declare_parameter<double>("homing_second_pass_torque_nm", 0.3);
    node->declare_parameter<double>("homing_stop_verification_torque_nm", 1.0);
    node->declare_parameter<int>("homing_stop_verification_ramp_ms", 2000);
    node->declare_parameter<int>("homing_torque_ramp_ms", 500);
    node->declare_parameter<double>("homing_velocity_trip_rad_s", 12.0);
    node->declare_parameter<double>("homing_min_bus_voltage_v", 10.0);
    node->declare_parameter<double>("homing_stall_velocity_rad_s", 0.2);
    node->declare_parameter<int>("homing_stall_dwell_ms", 500);
    node->declare_parameter<double>("homing_max_travel_rad", 0.75);
    node->declare_parameter<int>("homing_timeout_ms", 8000);
    node->declare_parameter<double>("homing_backoff_rad", 0.03);
    node->declare_parameter<double>("homing_repeatability_rad", 0.015);
    node->declare_parameter<double>("homing_min_motion_rad", 0.005);
    node->declare_parameter<bool>("init_devices_zero", false);

    candleParams_S params;
    params.data_rate   = node->get_parameter("data_rate").as_string();
    params.bus         = node->get_parameter("bus").as_string();
    params.default_qos = node->get_parameter("default_qos").as_string();
    params.joint_name_prefix = node->get_parameter("joint_name_prefix").as_string();
    params.gripper_open_position_rad =
        node->get_parameter("gripper_open_position_rad").as_double();
    params.gripper_closed_position_rad =
        node->get_parameter("gripper_closed_position_rad").as_double();
    params.gripper_impedance_kp = node->get_parameter("gripper_impedance_kp").as_double();
    params.gripper_impedance_kd = node->get_parameter("gripper_impedance_kd").as_double();
    params.gripper_velocity_limit_rad_s =
        node->get_parameter("gripper_velocity_limit_rad_s").as_double();
    params.gripper_torque_limit_nm =
        node->get_parameter("gripper_torque_limit_nm").as_double();
    params.soft_close_fast_torque_limit_nm =
        node->get_parameter("soft_close_fast_torque_limit_nm").as_double();
    params.soft_close_slow_torque_limit_nm =
        node->get_parameter("soft_close_slow_torque_limit_nm").as_double();
    params.soft_close_profile_acceleration_rad_s2 =
        node->get_parameter("soft_close_profile_acceleration_rad_s2").as_double();
    params.soft_close_profile_deceleration_rad_s2 =
        node->get_parameter("soft_close_profile_deceleration_rad_s2").as_double();
    params.soft_close_closed_tol_rad =
        node->get_parameter("soft_close_closed_tol_rad").as_double();
    params.soft_close_fast_duration_ms =
        node->get_parameter("soft_close_fast_duration_ms").as_int();
    params.health_publish_period_ms =
        node->get_parameter("health_publish_period_ms").as_int();
    params.gripper_state_position_tolerance_rad =
        node->get_parameter("gripper_state_position_tolerance_rad").as_double();
    params.gripper_state_moving_velocity_rad_s =
        node->get_parameter("gripper_state_moving_velocity_rad_s").as_double();
    params.encoder_wrap_period_rad =
        node->get_parameter("encoder_wrap_period_rad").as_double();
    params.position_recovery_max_delta_rad =
        node->get_parameter("position_recovery_max_delta_rad").as_double();
    params.position_recovery_samples =
        node->get_parameter("position_recovery_samples").as_int();
    params.position_recovery_retry_ms =
        node->get_parameter("position_recovery_retry_ms").as_int();
    params.startup_position_policy =
        node->get_parameter("startup_position_policy").as_string();
    params.position_state_file = node->get_parameter("position_state_file").as_string();
    params.position_state_write_period_ms =
        node->get_parameter("position_state_write_period_ms").as_int();
    params.position_state_min_change_rad =
        node->get_parameter("position_state_min_change_rad").as_double();
    params.homing_direction_by_id =
        node->get_parameter("homing_direction_by_id").as_string();
    params.homing_torque_nm = node->get_parameter("homing_torque_nm").as_double();
    params.homing_second_pass_torque_nm =
        node->get_parameter("homing_second_pass_torque_nm").as_double();
    params.homing_stop_verification_torque_nm =
        node->get_parameter("homing_stop_verification_torque_nm").as_double();
    params.homing_stop_verification_ramp_ms =
        node->get_parameter("homing_stop_verification_ramp_ms").as_int();
    params.homing_torque_ramp_ms = node->get_parameter("homing_torque_ramp_ms").as_int();
    params.homing_velocity_trip_rad_s =
        node->get_parameter("homing_velocity_trip_rad_s").as_double();
    params.homing_min_bus_voltage_v =
        node->get_parameter("homing_min_bus_voltage_v").as_double();
    params.homing_stall_velocity_rad_s =
        node->get_parameter("homing_stall_velocity_rad_s").as_double();
    params.homing_stall_dwell_ms =
        node->get_parameter("homing_stall_dwell_ms").as_int();
    params.homing_max_travel_rad =
        node->get_parameter("homing_max_travel_rad").as_double();
    params.homing_timeout_ms = node->get_parameter("homing_timeout_ms").as_int();
    params.homing_backoff_rad = node->get_parameter("homing_backoff_rad").as_double();
    params.homing_repeatability_rad =
        node->get_parameter("homing_repeatability_rad").as_double();
    params.homing_min_motion_rad =
        node->get_parameter("homing_min_motion_rad").as_double();
    params.init_devices_zero = node->get_parameter("init_devices_zero").as_bool();
    return params;
}

inline std::shared_ptr<mab::Candle> createCandle(const candleParams_S& params)
{
    auto dataRate = mab::CANdleDatarate_E::CAN_DATARATE_1M;
    auto bus      = mab::candleTypes::busTypes_t::USB;

    if (params.data_rate == "2M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_2M;
    else if (params.data_rate == "5M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_5M;
    else if (params.data_rate == "8M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_8M;

    if (params.bus == "SPI")
        bus = mab::candleTypes::busTypes_t::SPI;

    return std::shared_ptr<mab::Candle>(mab::attachCandle(dataRate, bus));
}
