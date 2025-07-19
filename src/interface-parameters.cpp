/*
 * Copyright (c) 2025, SECORO
 *
 * Authors: Vamsi Kalagaturu
 */

#include "eddie-ros/interface.hpp"

void EddieRosInterface::declare_all_parameters() {
    rcl_interfaces::msg::ParameterDescriptor ethercat_if_desc_;
    ethercat_if_desc_.description = "EtherCAT interface name";
    ethercat_if_desc_.type        = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("ethernet_if", "eno1", ethercat_if_desc_);
    this->get_parameter("ethernet_if", param_ethernet_if);
    RCLCPP_INFO(get_logger(), "Set param 'ethernet_if' to: %s", param_ethernet_if.c_str());

    rcl_interfaces::msg::ParameterDescriptor arm_to_control_desc_;
    arm_to_control_desc_.description = "Which arm to control: 'left', 'right' or 'both'.";
    arm_to_control_desc_.type        = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("arm_to_control", "", arm_to_control_desc_);
    this->get_parameter("arm_to_control", param_arm_to_control);
    if (param_arm_to_control != "left" && param_arm_to_control != "right" && param_arm_to_control != "both") {
        RCLCPP_ERROR(get_logger(), "Invalid value for 'arm_to_control'. Expected 'left', 'right' or 'both'.");
        throw std::runtime_error("Invalid parameter value for 'arm_to_control'");
    } else {
        RCLCPP_INFO(get_logger(), "Set param 'arm_to_control' to: %s", param_arm_to_control.c_str());
    }
}

void EddieRosInterface::get_all_parameters() {
    this->get_parameter("ethernet_if", param_ethernet_if);
    this->get_parameter("arm_to_control", param_arm_to_control);
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