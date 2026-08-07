#pragma once
#include <string>

struct candleParams_S
{
    std::string bus;
    std::string data_rate;
    std::string default_qos;
    std::string joint_name_prefix;
    double      gripper_open_position_rad;
    double      gripper_closed_position_rad;
    double      gripper_open_gap_mm;
    double      gripper_closed_gap_mm;
    double      gripper_impedance_kp;
    double      gripper_impedance_kd;
    double      gripper_velocity_limit_rad_s;
    double      gripper_torque_limit_nm;
    double      soft_close_fast_velocity_rad_s;
    double      soft_close_slow_velocity_rad_s;
    double      soft_close_fast_torque_limit_nm;
    double      soft_close_slow_torque_limit_nm;
    double      soft_close_profile_acceleration_rad_s2;
    double      soft_close_profile_deceleration_rad_s2;
    double      soft_close_closed_tol_rad;
    int         soft_close_fast_duration_ms;
    bool        init_devices_zero;
    double      home_impedance_kp;
    double      home_impedance_kd;
    double      home_torque_limit_nm;
    double      home_velocity_limit_rad_s;
    double      home_step_rad;
    double      home_stall_velocity_rad_s;
    double      home_stall_position_eps_rad;
    double      home_stall_torque_nm;
    int         home_stall_hold_ms;
    int         home_timeout_ms;
    int         home_poll_ms;
};
