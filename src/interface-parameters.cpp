/*
 * Copyright (c) 2025, SECORO
 *
 * Authors: Vamsi Kalagaturu
 */

#include "eddie_ros/interface.hpp"

void EddieRosInterface::declare_all_parameters() {
    rcl_interfaces::msg::ParameterDescriptor ethercat_if_desc_;
    ethercat_if_desc_.description = "EtherCAT interface name";
    ethercat_if_desc_.type        = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("ethernet_if", "eno1", ethercat_if_desc_);
    this->get_parameter("ethernet_if", param_ethernet_if);
    RCLCPP_INFO(get_logger(), "Set param 'ethernet_if' to: %s", param_ethernet_if.c_str());

    rcl_interfaces::msg::ParameterDescriptor arm_select_desc_;
    arm_select_desc_.description = "Which arm to control: 'left', 'right' or 'both'.";
    arm_select_desc_.type        = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("arm_select", "", arm_select_desc_);
    this->get_parameter("arm_select", param_arm_select);
    if (param_arm_select != "left" && param_arm_select != "right" && param_arm_select != "both") {
        RCLCPP_ERROR(get_logger(), "Invalid value for 'arm_select'. Expected 'left', 'right' or 'both'.");
        throw std::runtime_error("Invalid parameter value for 'arm_select'");
    } else {
        RCLCPP_INFO(get_logger(), "Set param 'arm_select' to: %s", param_arm_select.c_str());
    }

    rcl_interfaces::msg::ParameterDescriptor ft_sensor_com_port_;
    ft_sensor_com_port_.description = "COM port for the FT sensor (e.g., '/dev/ttyUSB0')";
    ft_sensor_com_port_.type        = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("ft_sensor_com_port", "", ft_sensor_com_port_);
    this->get_parameter("ft_sensor_com_port", param_ft_sensor_com_port);
    RCLCPP_INFO(get_logger(), "Set param 'ft_sensor_com_port' to: %s", param_ft_sensor_com_port.c_str());
}

void EddieRosInterface::get_all_parameters() {
    this->get_parameter("ethernet_if", param_ethernet_if);
    this->get_parameter("arm_select", param_arm_select);
    this->get_parameter("ft_sensor_com_port", param_ft_sensor_com_port);
}

// rcl_interfaces::msg::SetParametersResult
// EddieRosInterface::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
//     rcl_interfaces::msg::SetParametersResult result;
//     result.successful = true;
//     result.reason     = "Success";

//     for (const auto &param : parameters) {
//         if (param.get_name() == "ethernet_if") {
//             this->param_ethernet_if = param.get_value<std::string>();
//         }
//     }

//     return result;
// }