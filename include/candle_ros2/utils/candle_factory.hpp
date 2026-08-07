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
    node->declare_parameter<double>("gripper_open_gap_mm", 120.0);
    node->declare_parameter<double>("gripper_closed_gap_mm", 0.0);
    node->declare_parameter<double>("gripper_impedance_kp", 12.5);
    node->declare_parameter<double>("gripper_impedance_kd", 0.05);
    node->declare_parameter<double>("gripper_velocity_limit_rad_s", 3.5);
    node->declare_parameter<double>("gripper_torque_limit_nm", 4.0);
    node->declare_parameter<double>("soft_close_fast_kp", 6.0);
    node->declare_parameter<double>("soft_close_fast_kd", 0.05);
    node->declare_parameter<double>("soft_close_slow_kp", 20.0);
    node->declare_parameter<double>("soft_close_slow_kd", 0.12);
    node->declare_parameter<double>("soft_close_fast_tol_rad", 0.015);
    node->declare_parameter<double>("soft_close_closed_tol_rad", 0.005);
    node->declare_parameter<double>("soft_close_target_offset_rad", 0.05);
    node->declare_parameter<int>("soft_close_fast_duration_ms", 500);
    node->declare_parameter<bool>("init_devices_zero", false);
    node->declare_parameter<double>("home_impedance_kp", 4.0);
    node->declare_parameter<double>("home_impedance_kd", 0.05);
    node->declare_parameter<double>("home_torque_limit_nm", 2.5);
    node->declare_parameter<double>("home_velocity_limit_rad_s", 1.0);
    node->declare_parameter<double>("home_step_rad", 0.05);
    node->declare_parameter<double>("home_stall_velocity_rad_s", 0.02);
    node->declare_parameter<double>("home_stall_position_eps_rad", 0.005);
    node->declare_parameter<double>("home_stall_torque_nm", 0.25);
    node->declare_parameter<int>("home_stall_hold_ms", 300);
    node->declare_parameter<int>("home_timeout_ms", 8000);
    node->declare_parameter<int>("home_poll_ms", 20);

    candleParams_S params;
    params.data_rate   = node->get_parameter("data_rate").as_string();
    params.bus         = node->get_parameter("bus").as_string();
    params.default_qos = node->get_parameter("default_qos").as_string();
    params.joint_name_prefix = node->get_parameter("joint_name_prefix").as_string();
    params.gripper_open_position_rad =
        node->get_parameter("gripper_open_position_rad").as_double();
    params.gripper_closed_position_rad =
        node->get_parameter("gripper_closed_position_rad").as_double();
    params.gripper_open_gap_mm = node->get_parameter("gripper_open_gap_mm").as_double();
    params.gripper_closed_gap_mm = node->get_parameter("gripper_closed_gap_mm").as_double();
    params.gripper_impedance_kp = node->get_parameter("gripper_impedance_kp").as_double();
    params.gripper_impedance_kd = node->get_parameter("gripper_impedance_kd").as_double();
    params.gripper_velocity_limit_rad_s =
        node->get_parameter("gripper_velocity_limit_rad_s").as_double();
    params.gripper_torque_limit_nm =
        node->get_parameter("gripper_torque_limit_nm").as_double();
    params.soft_close_fast_kp = node->get_parameter("soft_close_fast_kp").as_double();
    params.soft_close_fast_kd = node->get_parameter("soft_close_fast_kd").as_double();
    params.soft_close_slow_kp = node->get_parameter("soft_close_slow_kp").as_double();
    params.soft_close_slow_kd = node->get_parameter("soft_close_slow_kd").as_double();
    params.soft_close_fast_tol_rad =
        node->get_parameter("soft_close_fast_tol_rad").as_double();
    params.soft_close_closed_tol_rad =
        node->get_parameter("soft_close_closed_tol_rad").as_double();
    params.soft_close_target_offset_rad =
        node->get_parameter("soft_close_target_offset_rad").as_double();
    params.soft_close_fast_duration_ms =
        node->get_parameter("soft_close_fast_duration_ms").as_int();
    params.init_devices_zero = node->get_parameter("init_devices_zero").as_bool();
    params.home_impedance_kp = node->get_parameter("home_impedance_kp").as_double();
    params.home_impedance_kd = node->get_parameter("home_impedance_kd").as_double();
    params.home_torque_limit_nm = node->get_parameter("home_torque_limit_nm").as_double();
    params.home_velocity_limit_rad_s =
        node->get_parameter("home_velocity_limit_rad_s").as_double();
    params.home_step_rad = node->get_parameter("home_step_rad").as_double();
    params.home_stall_velocity_rad_s =
        node->get_parameter("home_stall_velocity_rad_s").as_double();
    params.home_stall_position_eps_rad =
        node->get_parameter("home_stall_position_eps_rad").as_double();
    params.home_stall_torque_nm = node->get_parameter("home_stall_torque_nm").as_double();
    params.home_stall_hold_ms = node->get_parameter("home_stall_hold_ms").as_int();
    params.home_timeout_ms = node->get_parameter("home_timeout_ms").as_int();
    params.home_poll_ms = node->get_parameter("home_poll_ms").as_int();
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
