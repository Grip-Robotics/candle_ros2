#pragma once
#include <string>
#include "candle_ros2/impedance_close.hpp"
#include "candle_ros2/impedance_home.hpp"

struct candleParams_S
{
    std::string bus;
    std::string usb_serial;
    std::string data_rate;
    std::string default_qos;
    std::string joint_name_prefix;
    double      gripper_open_position_rad;
    double      gripper_closed_position_rad;
    double      gripper_impedance_kp;
    double      gripper_impedance_kd;
    double      gripper_velocity_limit_rad_s;
    double      gripper_open_velocity_limit_rad_s;
    double      gripper_torque_limit_nm;
    double      soft_close_slow_torque_limit_nm;
    ImpedanceClose::Config close;
    ImpedanceHome::Config home;
    bool        homing_required;
    double      homing_travel_margin_rad = 0.5;
    int         joint_state_publish_period_ms;
    int         health_publish_period_ms;
    int         md_can_response_timeout_100us;
    int         md_host_timeout_ms;
    double      encoder_wrap_period_rad;
    double      position_recovery_max_delta_rad;
    int         position_recovery_samples;
    int         position_recovery_retry_ms;
    bool        init_devices_zero;
};
