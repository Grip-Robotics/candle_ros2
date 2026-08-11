# md_node_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bus_arg = DeclareLaunchArgument(
        "bus",
        default_value="USB",
        description="Bus type: USB or SPI",
    )

    data_rate_arg = DeclareLaunchArgument(
        "data_rate",
        default_value="1M",
        description="Data rate: 1M, 2M, 5M or 8M",
    )

    default_qos_arg = DeclareLaunchArgument(
        "default_qos",
        default_value="Reliable",
        description='Quality of Service: "BestEffort" or "Reliable"',
    )

    gripper_args = [
        DeclareLaunchArgument("joint_name_prefix", default_value="md_"),
        DeclareLaunchArgument("gripper_open_position_rad", default_value="0.0"),
        DeclareLaunchArgument("gripper_closed_position_rad", default_value="0.63"),
        DeclareLaunchArgument("gripper_impedance_kp", default_value="12.5"),
        DeclareLaunchArgument("gripper_impedance_kd", default_value="0.05"),
        DeclareLaunchArgument("gripper_velocity_limit_rad_s", default_value="6.0"),
        DeclareLaunchArgument("gripper_torque_limit_nm", default_value="3.0"),
        DeclareLaunchArgument("soft_close_fast_torque_limit_nm", default_value="4.0"),
        DeclareLaunchArgument("soft_close_slow_torque_limit_nm", default_value="4.0"),
        DeclareLaunchArgument("soft_close_profile_acceleration_rad_s2", default_value="100.0"),
        DeclareLaunchArgument("soft_close_profile_deceleration_rad_s2", default_value="100.0"),
        DeclareLaunchArgument("soft_close_closed_tol_rad", default_value="0.005"),
        DeclareLaunchArgument("soft_close_fast_duration_ms", default_value="300"),
        DeclareLaunchArgument("health_publish_period_ms", default_value="200"),
        DeclareLaunchArgument("encoder_wrap_period_rad", default_value="0.62831853"),
        DeclareLaunchArgument("position_recovery_max_delta_rad", default_value="0.25"),
        DeclareLaunchArgument("position_recovery_samples", default_value="3"),
        DeclareLaunchArgument("position_recovery_retry_ms", default_value="25"),
        DeclareLaunchArgument("startup_position_policy", default_value="restore_or_home"),
        DeclareLaunchArgument(
            "position_state_file",
            default_value="~/.ros/candle_ros2_position_state.json",
        ),
        DeclareLaunchArgument("position_state_write_period_ms", default_value="1000"),
        DeclareLaunchArgument("position_state_min_change_rad", default_value="0.005"),
        DeclareLaunchArgument(
            "homing_direction_by_id", default_value="342:-1,343:-1,345:-1"
        ),
        DeclareLaunchArgument("homing_torque_nm", default_value="0.3"),
        DeclareLaunchArgument("homing_second_pass_torque_nm", default_value="0.3"),
        DeclareLaunchArgument(
            "homing_stop_verification_torque_nm", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "homing_stop_verification_ramp_ms", default_value="2000"
        ),
        DeclareLaunchArgument("homing_torque_ramp_ms", default_value="500"),
        DeclareLaunchArgument("homing_velocity_trip_rad_s", default_value="12.0"),
        DeclareLaunchArgument("homing_min_bus_voltage_v", default_value="10.0"),
        DeclareLaunchArgument("homing_stall_velocity_rad_s", default_value="0.2"),
        DeclareLaunchArgument("homing_stall_dwell_ms", default_value="500"),
        DeclareLaunchArgument("homing_max_travel_rad", default_value="0.75"),
        DeclareLaunchArgument("homing_timeout_ms", default_value="8000"),
        DeclareLaunchArgument("homing_backoff_rad", default_value="0.03"),
        DeclareLaunchArgument("homing_repeatability_rad", default_value="0.015"),
        DeclareLaunchArgument("homing_min_motion_rad", default_value="0.005"),
        DeclareLaunchArgument("init_devices_zero", default_value="false"),
    ]

    bus = LaunchConfiguration("bus")
    data_rate = LaunchConfiguration("data_rate")
    default_qos = LaunchConfiguration("default_qos")

    return LaunchDescription(
        [
            bus_arg,
            data_rate_arg,
            default_qos_arg,
            *gripper_args,
            Node(
                package="candle_ros2",
                executable="candle_container",
                output="screen",
                parameters=[
                    {
                        "launch_md_node": True,
                        "launch_pds_node": False,
                        "bus": bus,
                        "data_rate": data_rate,
                        "default_qos": default_qos,
                        "joint_name_prefix": LaunchConfiguration("joint_name_prefix"),
                        "gripper_open_position_rad": ParameterValue(
                            LaunchConfiguration("gripper_open_position_rad"), value_type=float
                        ),
                        "gripper_closed_position_rad": ParameterValue(
                            LaunchConfiguration("gripper_closed_position_rad"), value_type=float
                        ),
                        "gripper_impedance_kp": ParameterValue(
                            LaunchConfiguration("gripper_impedance_kp"), value_type=float
                        ),
                        "gripper_impedance_kd": ParameterValue(
                            LaunchConfiguration("gripper_impedance_kd"), value_type=float
                        ),
                        "gripper_velocity_limit_rad_s": ParameterValue(
                            LaunchConfiguration("gripper_velocity_limit_rad_s"), value_type=float
                        ),
                        "gripper_torque_limit_nm": ParameterValue(
                            LaunchConfiguration("gripper_torque_limit_nm"), value_type=float
                        ),
                        "soft_close_fast_torque_limit_nm": ParameterValue(
                            LaunchConfiguration("soft_close_fast_torque_limit_nm"),
                            value_type=float,
                        ),
                        "soft_close_slow_torque_limit_nm": ParameterValue(
                            LaunchConfiguration("soft_close_slow_torque_limit_nm"),
                            value_type=float,
                        ),
                        "soft_close_profile_acceleration_rad_s2": ParameterValue(
                            LaunchConfiguration("soft_close_profile_acceleration_rad_s2"),
                            value_type=float,
                        ),
                        "soft_close_profile_deceleration_rad_s2": ParameterValue(
                            LaunchConfiguration("soft_close_profile_deceleration_rad_s2"),
                            value_type=float,
                        ),
                        "soft_close_closed_tol_rad": ParameterValue(
                            LaunchConfiguration("soft_close_closed_tol_rad"), value_type=float
                        ),
                        "soft_close_fast_duration_ms": ParameterValue(
                            LaunchConfiguration("soft_close_fast_duration_ms"), value_type=int
                        ),
                        "health_publish_period_ms": ParameterValue(
                            LaunchConfiguration("health_publish_period_ms"), value_type=int
                        ),
                        "encoder_wrap_period_rad": ParameterValue(
                            LaunchConfiguration("encoder_wrap_period_rad"), value_type=float
                        ),
                        "position_recovery_max_delta_rad": ParameterValue(
                            LaunchConfiguration("position_recovery_max_delta_rad"),
                            value_type=float,
                        ),
                        "position_recovery_samples": ParameterValue(
                            LaunchConfiguration("position_recovery_samples"), value_type=int
                        ),
                        "position_recovery_retry_ms": ParameterValue(
                            LaunchConfiguration("position_recovery_retry_ms"), value_type=int
                        ),
                        "startup_position_policy": LaunchConfiguration(
                            "startup_position_policy"
                        ),
                        "position_state_file": LaunchConfiguration("position_state_file"),
                        "position_state_write_period_ms": ParameterValue(
                            LaunchConfiguration("position_state_write_period_ms"),
                            value_type=int,
                        ),
                        "position_state_min_change_rad": ParameterValue(
                            LaunchConfiguration("position_state_min_change_rad"),
                            value_type=float,
                        ),
                        "homing_direction_by_id": LaunchConfiguration(
                            "homing_direction_by_id"
                        ),
                        "homing_torque_nm": ParameterValue(
                            LaunchConfiguration("homing_torque_nm"), value_type=float
                        ),
                        "homing_second_pass_torque_nm": ParameterValue(
                            LaunchConfiguration("homing_second_pass_torque_nm"),
                            value_type=float,
                        ),
                        "homing_stop_verification_torque_nm": ParameterValue(
                            LaunchConfiguration(
                                "homing_stop_verification_torque_nm"
                            ),
                            value_type=float,
                        ),
                        "homing_stop_verification_ramp_ms": ParameterValue(
                            LaunchConfiguration(
                                "homing_stop_verification_ramp_ms"
                            ),
                            value_type=int,
                        ),
                        "homing_torque_ramp_ms": ParameterValue(
                            LaunchConfiguration("homing_torque_ramp_ms"), value_type=int
                        ),
                        "homing_velocity_trip_rad_s": ParameterValue(
                            LaunchConfiguration("homing_velocity_trip_rad_s"),
                            value_type=float,
                        ),
                        "homing_min_bus_voltage_v": ParameterValue(
                            LaunchConfiguration("homing_min_bus_voltage_v"),
                            value_type=float,
                        ),
                        "homing_stall_velocity_rad_s": ParameterValue(
                            LaunchConfiguration("homing_stall_velocity_rad_s"),
                            value_type=float,
                        ),
                        "homing_stall_dwell_ms": ParameterValue(
                            LaunchConfiguration("homing_stall_dwell_ms"), value_type=int
                        ),
                        "homing_max_travel_rad": ParameterValue(
                            LaunchConfiguration("homing_max_travel_rad"), value_type=float
                        ),
                        "homing_timeout_ms": ParameterValue(
                            LaunchConfiguration("homing_timeout_ms"), value_type=int
                        ),
                        "homing_backoff_rad": ParameterValue(
                            LaunchConfiguration("homing_backoff_rad"), value_type=float
                        ),
                        "homing_repeatability_rad": ParameterValue(
                            LaunchConfiguration("homing_repeatability_rad"),
                            value_type=float,
                        ),
                        "homing_min_motion_rad": ParameterValue(
                            LaunchConfiguration("homing_min_motion_rad"), value_type=float
                        ),
                        "init_devices_zero": ParameterValue(
                            LaunchConfiguration("init_devices_zero"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
