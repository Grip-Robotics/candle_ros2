#pragma once
#include <string>

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
    double      gripper_torque_limit_nm;
    double      soft_close_fast_torque_limit_nm;
    double      soft_close_slow_torque_limit_nm;
    double      soft_close_profile_acceleration_rad_s2;
    double      soft_close_profile_deceleration_rad_s2;
    double      soft_close_closed_tol_rad;
    int         soft_close_fast_duration_ms;
    int         health_publish_period_ms;
    double      encoder_wrap_period_rad;
    double      position_recovery_max_delta_rad;
    int         position_recovery_samples;
    int         position_recovery_retry_ms;
    bool        init_devices_zero;
};
