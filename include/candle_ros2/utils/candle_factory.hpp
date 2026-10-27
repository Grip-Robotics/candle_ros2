#pragma once
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

/* Utils */
#include "candle_ros2/utils/candle_params.hpp"

/* CANdle-SDK */
#include "candle.hpp"

namespace candle_ros2::detail
{
class HostTimeoutBoundedInterface final : public mab::I_CommunicationInterface
{
  public:
    HostTimeoutBoundedInterface(
        std::unique_ptr<mab::I_CommunicationInterface>&& interface, u32 hostTimeoutMs)
        : m_interface(std::move(interface)), m_hostTimeoutMs(hostTimeoutMs)
    {
    }

    Error_t connect() override
    {
        return m_interface->connect();
    }

    Error_t disconnect() override
    {
        return m_interface->disconnect();
    }

    Error_t transfer(std::vector<u8> data, const u32 timeoutMs) override
    {
        return m_interface->transfer(
            std::move(data), std::min(timeoutMs, m_hostTimeoutMs));
    }

    std::pair<std::vector<u8>, Error_t> transfer(
        std::vector<u8> data,
        const u32       timeoutMs,
        const size_t    expectedReceivedDataSize) override
    {
        return m_interface->transfer(std::move(data),
                                     std::min(timeoutMs, m_hostTimeoutMs),
                                     expectedReceivedDataSize);
    }

  private:
    std::unique_ptr<mab::I_CommunicationInterface> m_interface;
    u32                                            m_hostTimeoutMs;
};
}  // namespace candle_ros2::detail

inline candleParams_S readParams(const rclcpp::Node::SharedPtr& node)
{
    node->declare_parameter<std::string>("data_rate", "1M");
    node->declare_parameter<std::string>("bus", "USB");
    node->declare_parameter<std::string>("usb_serial", "");
    node->declare_parameter<std::string>("default_qos", "Reliable");
    node->declare_parameter<std::string>("joint_name_prefix", "md_");
    node->declare_parameter<double>("gripper_open_position_rad", 0.0);
    node->declare_parameter<double>("gripper_closed_position_rad", 0.62);
    node->declare_parameter<double>("gripper_impedance_kp", 12.5);
    node->declare_parameter<double>("gripper_impedance_kd", 0.05);
    node->declare_parameter<double>("gripper_velocity_limit_rad_s", 3.5);
    node->declare_parameter<double>("gripper_torque_limit_nm", 4.0);
    node->declare_parameter<double>("soft_close_fast_torque_limit_nm", 4.0);
    node->declare_parameter<double>("soft_close_slow_torque_limit_nm", 4.0);
    node->declare_parameter<double>("soft_close_profile_acceleration_rad_s2", 100.0);
    node->declare_parameter<double>("soft_close_profile_deceleration_rad_s2", 100.0);
    node->declare_parameter<double>("soft_close_closed_tol_rad", 0.005);
    node->declare_parameter<int>("soft_close_fast_duration_ms", 300);
    node->declare_parameter<int>("joint_state_publish_period_ms", 50);
    node->declare_parameter<int>("health_publish_period_ms", 200);
    node->declare_parameter<int>("md_can_response_timeout_100us", 100);
    node->declare_parameter<int>("md_host_timeout_ms", 20);
    node->declare_parameter<double>("encoder_wrap_period_rad", 0.62831853);
    node->declare_parameter<double>("position_recovery_max_delta_rad", 0.25);
    node->declare_parameter<int>("position_recovery_samples", 3);
    node->declare_parameter<int>("position_recovery_retry_ms", 25);
    node->declare_parameter<bool>("init_devices_zero", false);

    candleParams_S params;
    params.data_rate   = node->get_parameter("data_rate").as_string();
    params.bus         = node->get_parameter("bus").as_string();
    params.usb_serial  = node->get_parameter("usb_serial").as_string();
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
    params.joint_state_publish_period_ms =
        node->get_parameter("joint_state_publish_period_ms").as_int();
    params.health_publish_period_ms =
        node->get_parameter("health_publish_period_ms").as_int();
    params.md_can_response_timeout_100us =
        node->get_parameter("md_can_response_timeout_100us").as_int();
    params.md_host_timeout_ms =
        node->get_parameter("md_host_timeout_ms").as_int();
    params.encoder_wrap_period_rad =
        node->get_parameter("encoder_wrap_period_rad").as_double();
    params.position_recovery_max_delta_rad =
        node->get_parameter("position_recovery_max_delta_rad").as_double();
    params.position_recovery_samples =
        node->get_parameter("position_recovery_samples").as_int();
    params.position_recovery_retry_ms =
        node->get_parameter("position_recovery_retry_ms").as_int();
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

    if (bus == mab::candleTypes::busTypes_t::USB)
    {
        if (params.md_host_timeout_ms <= 0)
            throw std::invalid_argument("md_host_timeout_ms must be positive");
        std::unique_ptr<mab::I_CommunicationInterface> rawUsb =
            std::make_unique<mab::USB>(mab::Candle::CANDLE_VID,
                                       mab::Candle::CANDLE_PID,
                                       params.usb_serial);
        std::unique_ptr<mab::I_CommunicationInterface> usb =
            std::make_unique<candle_ros2::detail::HostTimeoutBoundedInterface>(
                std::move(rawUsb),
                static_cast<u32>(params.md_host_timeout_ms));
        if (usb->connect() != mab::I_CommunicationInterface::Error_t::OK)
            throw std::runtime_error("Could not connect selected USB device!");
        return std::shared_ptr<mab::Candle>(
            mab::attachCandle(dataRate, std::move(usb)));
    }

    return std::shared_ptr<mab::Candle>(mab::attachCandle(dataRate, bus));
}
