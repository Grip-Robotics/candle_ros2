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

    usb_serial_arg = DeclareLaunchArgument(
        "usb_serial",
        default_value="",
        description="Complete USB serial of the CANdle adapter",
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
        DeclareLaunchArgument("gripper_closed_position_rad", default_value="0.75"),
        DeclareLaunchArgument("gripper_impedance_kp", default_value="12.5"),
        DeclareLaunchArgument("gripper_impedance_kd", default_value="0.05"),
        DeclareLaunchArgument("gripper_impedance_ki", default_value="2.0"),
        DeclareLaunchArgument("gripper_hold_torque_nm", default_value="2.0"),
        DeclareLaunchArgument("gripper_closed_hold_torque_nm", default_value="0.25"),
        DeclareLaunchArgument("gripper_hold_torque_ramp_nm_s", default_value="2.0"),
        DeclareLaunchArgument("close_object_margin_rad", default_value="0.01"),
        DeclareLaunchArgument("gripper_velocity_limit_rad_s", default_value="1.5"),
        DeclareLaunchArgument("gripper_open_velocity_limit_rad_s", default_value="10.0"),
        DeclareLaunchArgument("gripper_torque_limit_nm", default_value="2.0"),
        DeclareLaunchArgument("soft_close_fast_torque_limit_nm", default_value="4.0"),
        DeclareLaunchArgument("soft_close_slow_torque_limit_nm", default_value="2.0"),
        DeclareLaunchArgument("soft_close_profile_acceleration_rad_s2", default_value="100.0"),
        DeclareLaunchArgument("soft_close_profile_deceleration_rad_s2", default_value="100.0"),
        DeclareLaunchArgument("soft_close_closed_tol_rad", default_value="0.005"),
        DeclareLaunchArgument("gripper_travel_min_rad", default_value="-0.05"),
        DeclareLaunchArgument("gripper_travel_max_rad", default_value="0.75"),
        DeclareLaunchArgument("gripper_travel_limits_enabled", default_value="true"),
        DeclareLaunchArgument("soft_close_fast_duration_ms", default_value="300"),
        DeclareLaunchArgument("close_target_lead_rad", default_value="0.04"),
        DeclareLaunchArgument("open_target_lead_rad",
                              default_value=LaunchConfiguration("close_target_lead_rad")),
        DeclareLaunchArgument("close_contact_torque_nm", default_value="0.35"),
        DeclareLaunchArgument("close_contact_velocity_rad_s", default_value="0.03"),
        DeclareLaunchArgument("close_contact_duration_s", default_value="0.15"),
        DeclareLaunchArgument("close_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("open_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("open_obstruction_tolerance_rad", default_value="0.017453292519943295"),
        DeclareLaunchArgument("close_feedback_timeout_s", default_value="0.25"),
        DeclareLaunchArgument("joint_state_publish_period_ms", default_value="50"),
        DeclareLaunchArgument("health_publish_period_ms", default_value="200"),
        DeclareLaunchArgument("md_can_response_timeout_100us", default_value="100"),
        DeclareLaunchArgument("md_host_timeout_ms", default_value="20"),
        DeclareLaunchArgument("encoder_wrap_period_rad", default_value="0.62831853"),
        DeclareLaunchArgument("position_recovery_max_delta_rad", default_value="0.25"),
        DeclareLaunchArgument("position_recovery_samples", default_value="3"),
        DeclareLaunchArgument("position_recovery_retry_ms", default_value="25"),
        DeclareLaunchArgument("init_devices_zero", default_value="false"),
        DeclareLaunchArgument("homing_required", default_value="false"),
        DeclareLaunchArgument("homing_torque_nm", default_value="1.0"),
        DeclareLaunchArgument("homing_damping_nm_s_rad", default_value="0.5"),
        DeclareLaunchArgument("homing_torque_limit_nm", default_value="2.0"),
        DeclareLaunchArgument("homing_travel_margin_rad", default_value="0.5"),
        DeclareLaunchArgument("homing_contact_duration_s", default_value="0.25"),
        DeclareLaunchArgument("homing_timeout_s", default_value="2.0"),
    ]

    bus = LaunchConfiguration("bus")
    usb_serial = LaunchConfiguration("usb_serial")
    data_rate = LaunchConfiguration("data_rate")
    default_qos = LaunchConfiguration("default_qos")

    return LaunchDescription(
        [
            bus_arg,
            usb_serial_arg,
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
                        "usb_serial": ParameterValue(usb_serial, value_type=str),
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
                        "gripper_impedance_ki": ParameterValue(
                            LaunchConfiguration("gripper_impedance_ki"), value_type=float
                        ),
                        "gripper_hold_torque_nm": ParameterValue(
                            LaunchConfiguration("gripper_hold_torque_nm"), value_type=float
                        ),
                        "gripper_closed_hold_torque_nm": ParameterValue(
                            LaunchConfiguration("gripper_closed_hold_torque_nm"), value_type=float
                        ),
                        "gripper_hold_torque_ramp_nm_s": ParameterValue(
                            LaunchConfiguration("gripper_hold_torque_ramp_nm_s"), value_type=float
                        ),
                        "close_object_margin_rad": ParameterValue(
                            LaunchConfiguration("close_object_margin_rad"), value_type=float
                        ),
                        "gripper_velocity_limit_rad_s": ParameterValue(
                            LaunchConfiguration("gripper_velocity_limit_rad_s"), value_type=float
                        ),
                        "gripper_open_velocity_limit_rad_s": ParameterValue(
                            LaunchConfiguration("gripper_open_velocity_limit_rad_s"),
                            value_type=float,
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
                        "gripper_travel_min_rad": ParameterValue(
                            LaunchConfiguration("gripper_travel_min_rad"), value_type=float
                        ),
                        "gripper_travel_max_rad": ParameterValue(
                            LaunchConfiguration("gripper_travel_max_rad"), value_type=float
                        ),
                        "gripper_travel_limits_enabled": ParameterValue(
                            LaunchConfiguration("gripper_travel_limits_enabled"), value_type=bool
                        ),
                        "soft_close_fast_duration_ms": ParameterValue(
                            LaunchConfiguration("soft_close_fast_duration_ms"), value_type=int
                        ),
                        "close_target_lead_rad": ParameterValue(
                            LaunchConfiguration("close_target_lead_rad"), value_type=float
                        ),
                        "open_target_lead_rad": ParameterValue(
                            LaunchConfiguration("open_target_lead_rad"), value_type=float
                        ),
                        "close_contact_torque_nm": ParameterValue(
                            LaunchConfiguration("close_contact_torque_nm"), value_type=float
                        ),
                        "close_contact_velocity_rad_s": ParameterValue(
                            LaunchConfiguration("close_contact_velocity_rad_s"), value_type=float
                        ),
                        "close_contact_duration_s": ParameterValue(
                            LaunchConfiguration("close_contact_duration_s"), value_type=float
                        ),
                        "close_timeout_s": ParameterValue(
                            LaunchConfiguration("close_timeout_s"), value_type=float
                        ),
                        "open_obstruction_tolerance_rad": ParameterValue(
                            LaunchConfiguration("open_obstruction_tolerance_rad"), value_type=float
                        ),
                        "open_timeout_s": ParameterValue(
                            LaunchConfiguration("open_timeout_s"), value_type=float
                        ),
                        "close_feedback_timeout_s": ParameterValue(
                            LaunchConfiguration("close_feedback_timeout_s"), value_type=float
                        ),
                        "joint_state_publish_period_ms": ParameterValue(
                            LaunchConfiguration("joint_state_publish_period_ms"), value_type=int
                        ),
                        "health_publish_period_ms": ParameterValue(
                            LaunchConfiguration("health_publish_period_ms"), value_type=int
                        ),
                        "md_can_response_timeout_100us": ParameterValue(
                            LaunchConfiguration("md_can_response_timeout_100us"), value_type=int
                        ),
                        "md_host_timeout_ms": ParameterValue(
                            LaunchConfiguration("md_host_timeout_ms"), value_type=int
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
                        "init_devices_zero": ParameterValue(
                            LaunchConfiguration("init_devices_zero"), value_type=bool
                        ),
                        "homing_required": ParameterValue(
                            LaunchConfiguration("homing_required"), value_type=bool
                        ),
                        **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                           for name in ("homing_torque_nm", "homing_damping_nm_s_rad", "homing_torque_limit_nm",
                                        "homing_travel_margin_rad",
                                        "homing_contact_duration_s", "homing_timeout_s")},
                    }
                ],
            ),
        ]
    )
