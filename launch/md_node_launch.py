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
        DeclareLaunchArgument("gripper_closed_position_rad", default_value="0.62"),
        DeclareLaunchArgument("gripper_open_gap_mm", default_value="120.0"),
        DeclareLaunchArgument("gripper_closed_gap_mm", default_value="0.0"),
        DeclareLaunchArgument("gripper_impedance_kp", default_value="12.5"),
        DeclareLaunchArgument("gripper_impedance_kd", default_value="0.05"),
        DeclareLaunchArgument("gripper_velocity_limit_rad_s", default_value="3.5"),
        DeclareLaunchArgument("gripper_torque_limit_nm", default_value="4.0"),
        DeclareLaunchArgument("soft_close_fast_velocity_rad_s", default_value="2.0"),
        DeclareLaunchArgument("soft_close_slow_velocity_rad_s", default_value="0.3"),
        DeclareLaunchArgument("soft_close_fast_torque_limit_nm", default_value="3.0"),
        DeclareLaunchArgument("soft_close_slow_torque_limit_nm", default_value="3.0"),
        DeclareLaunchArgument("soft_close_profile_acceleration_rad_s2", default_value="5.0"),
        DeclareLaunchArgument("soft_close_profile_deceleration_rad_s2", default_value="5.0"),
        DeclareLaunchArgument("soft_close_closed_tol_rad", default_value="0.005"),
        DeclareLaunchArgument("soft_close_fast_duration_ms", default_value="500"),
        DeclareLaunchArgument("init_devices_zero", default_value="false"),
        DeclareLaunchArgument("home_impedance_kp", default_value="4.0"),
        DeclareLaunchArgument("home_impedance_kd", default_value="0.05"),
        DeclareLaunchArgument("home_torque_limit_nm", default_value="2.5"),
        DeclareLaunchArgument("home_velocity_limit_rad_s", default_value="1.0"),
        DeclareLaunchArgument("home_step_rad", default_value="0.05"),
        DeclareLaunchArgument("home_stall_velocity_rad_s", default_value="0.02"),
        DeclareLaunchArgument("home_stall_position_eps_rad", default_value="0.005"),
        DeclareLaunchArgument("home_stall_torque_nm", default_value="0.25"),
        DeclareLaunchArgument("home_stall_hold_ms", default_value="300"),
        DeclareLaunchArgument("home_timeout_ms", default_value="8000"),
        DeclareLaunchArgument("home_poll_ms", default_value="20"),
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
                        "gripper_open_gap_mm": ParameterValue(
                            LaunchConfiguration("gripper_open_gap_mm"), value_type=float
                        ),
                        "gripper_closed_gap_mm": ParameterValue(
                            LaunchConfiguration("gripper_closed_gap_mm"), value_type=float
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
                        "soft_close_fast_velocity_rad_s": ParameterValue(
                            LaunchConfiguration("soft_close_fast_velocity_rad_s"),
                            value_type=float,
                        ),
                        "soft_close_slow_velocity_rad_s": ParameterValue(
                            LaunchConfiguration("soft_close_slow_velocity_rad_s"),
                            value_type=float,
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
                        "init_devices_zero": ParameterValue(
                            LaunchConfiguration("init_devices_zero"), value_type=bool
                        ),
                        "home_impedance_kp": ParameterValue(
                            LaunchConfiguration("home_impedance_kp"), value_type=float
                        ),
                        "home_impedance_kd": ParameterValue(
                            LaunchConfiguration("home_impedance_kd"), value_type=float
                        ),
                        "home_torque_limit_nm": ParameterValue(
                            LaunchConfiguration("home_torque_limit_nm"), value_type=float
                        ),
                        "home_velocity_limit_rad_s": ParameterValue(
                            LaunchConfiguration("home_velocity_limit_rad_s"), value_type=float
                        ),
                        "home_step_rad": ParameterValue(
                            LaunchConfiguration("home_step_rad"), value_type=float
                        ),
                        "home_stall_velocity_rad_s": ParameterValue(
                            LaunchConfiguration("home_stall_velocity_rad_s"), value_type=float
                        ),
                        "home_stall_position_eps_rad": ParameterValue(
                            LaunchConfiguration("home_stall_position_eps_rad"), value_type=float
                        ),
                        "home_stall_torque_nm": ParameterValue(
                            LaunchConfiguration("home_stall_torque_nm"), value_type=float
                        ),
                        "home_stall_hold_ms": ParameterValue(
                            LaunchConfiguration("home_stall_hold_ms"), value_type=int
                        ),
                        "home_timeout_ms": ParameterValue(
                            LaunchConfiguration("home_timeout_ms"), value_type=int
                        ),
                        "home_poll_ms": ParameterValue(
                            LaunchConfiguration("home_poll_ms"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
