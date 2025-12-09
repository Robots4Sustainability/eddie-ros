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
}

void EddieRosInterface::get_all_parameters() {
    this->get_parameter("ethernet_if", param_ethernet_if);
    this->get_parameter("arm_select", param_arm_select);
}

void EddieRosInterface::declare_pid_gains() {

    RCLCPP_INFO(this->get_logger(), "Declaring PID gains from parameter server...");

    // Torque smoothing parameter
    this->declare_parameter<double>("torque_smoothing_alpha", 0.05);

    const double default_pos_deadband = 0.005;
    const double default_rot_deadband = 0.02;

    // Declare parameters for the RIGHT arm
    this->declare_parameter<double>("pid.right.pos.x.p", 80.0);
    this->declare_parameter<double>("pid.right.pos.x.i", 20.0);
    this->declare_parameter<double>("pid.right.pos.x.d", 10.0);
    this->declare_parameter<double>("pid.right.pos.y.p", 150.0);
    this->declare_parameter<double>("pid.right.pos.y.i", 20.0);
    this->declare_parameter<double>("pid.right.pos.y.d", 10.0);
    this->declare_parameter<double>("pid.right.pos.z.p", 150.0);
    this->declare_parameter<double>("pid.right.pos.z.i", 20.0);
    this->declare_parameter<double>("pid.right.pos.z.d", 10.0);
    this->declare_parameter<double>("pid.right.pos.deadband", default_pos_deadband);

    this->declare_parameter<double>("pid.right.rot.x.p", 5.0);
    this->declare_parameter<double>("pid.right.rot.x.i", 0.0);
    this->declare_parameter<double>("pid.right.rot.x.d", 2.0);
    this->declare_parameter<double>("pid.right.rot.y.p", 5.0);
    this->declare_parameter<double>("pid.right.rot.y.i", 0.0);
    this->declare_parameter<double>("pid.right.rot.y.d", 2.0);
    this->declare_parameter<double>("pid.right.rot.z.p", 5.0);
    this->declare_parameter<double>("pid.right.rot.z.i", 0.0);
    this->declare_parameter<double>("pid.right.rot.z.d", 2.0);
    this->declare_parameter<double>("pid.right.rot.deadband", default_rot_deadband);

    // Declare parameters for the LEFT arm
    this->declare_parameter<double>("pid.left.pos.x.p", 70.0);
    this->declare_parameter<double>("pid.left.pos.x.i", 0.0);
    this->declare_parameter<double>("pid.left.pos.x.d", 4.0);
    this->declare_parameter<double>("pid.left.pos.y.p", 70.0);
    this->declare_parameter<double>("pid.left.pos.y.i", 0.0);
    this->declare_parameter<double>("pid.left.pos.y.d", 4.0);
    this->declare_parameter<double>("pid.left.pos.z.p", 150.0);
    this->declare_parameter<double>("pid.left.pos.z.i", 8.0);
    this->declare_parameter<double>("pid.left.pos.z.d", 10.0);
    this->declare_parameter<double>("pid.left.pos.deadband", default_pos_deadband);

    this->declare_parameter<double>("pid.left.rot.x.p", 5.0);
    this->declare_parameter<double>("pid.left.rot.x.i", 0.0);
    this->declare_parameter<double>("pid.left.rot.x.d", 2.0);
    this->declare_parameter<double>("pid.left.rot.y.p", 5.0);
    this->declare_parameter<double>("pid.left.rot.y.i", 0.0);
    this->declare_parameter<double>("pid.left.rot.y.d", 2.0);
    this->declare_parameter<double>("pid.left.rot.z.p", 5.0);
    this->declare_parameter<double>("pid.left.rot.z.i", 0.0);
    this->declare_parameter<double>("pid.left.rot.z.d", 2.0);
    this->declare_parameter<double>("pid.left.rot.deadband", default_rot_deadband);
}

void EddieRosInterface::reload_pid_gains()
{
    RCLCPP_INFO(this->get_logger(), "Reloading all PID gains from parameter server...");

    const double error_sum_tol = 0.9;
    const double decay_rate = 0.0;

    // Get values for the RIGHT arm
    double r_pos_x_p = this->get_parameter("pid.right.pos.x.p").as_double();
    double r_pos_x_i = this->get_parameter("pid.right.pos.x.i").as_double();
    double r_pos_x_d = this->get_parameter("pid.right.pos.x.d").as_double();
    double r_pos_y_p = this->get_parameter("pid.right.pos.y.p").as_double();
    double r_pos_y_i = this->get_parameter("pid.right.pos.y.i").as_double();
    double r_pos_y_d = this->get_parameter("pid.right.pos.y.d").as_double();
    double r_pos_z_p = this->get_parameter("pid.right.pos.z.p").as_double();
    double r_pos_z_i = this->get_parameter("pid.right.pos.z.i").as_double();
    double r_pos_z_d = this->get_parameter("pid.right.pos.z.d").as_double();
    double r_pos_deadband = this->get_parameter("pid.right.pos.deadband").as_double();
    
    double r_rot_x_p = this->get_parameter("pid.right.rot.x.p").as_double();
    double r_rot_x_i = this->get_parameter("pid.right.rot.x.i").as_double();
    double r_rot_x_d = this->get_parameter("pid.right.rot.x.d").as_double();
    double r_rot_y_p = this->get_parameter("pid.right.rot.y.p").as_double();
    double r_rot_y_i = this->get_parameter("pid.right.rot.y.i").as_double();
    double r_rot_y_d = this->get_parameter("pid.right.rot.y.d").as_double();
    double r_rot_z_p = this->get_parameter("pid.right.rot.z.p").as_double();
    double r_rot_z_i = this->get_parameter("pid.right.rot.z.i").as_double();
    double r_rot_z_d = this->get_parameter("pid.right.rot.z.d").as_double();
    double r_rot_deadband = this->get_parameter("pid.right.rot.deadband").as_double();

    // Get values for the LEFT arm
    double l_pos_x_p = this->get_parameter("pid.left.pos.x.p").as_double();
    double l_pos_x_i = this->get_parameter("pid.left.pos.x.i").as_double();
    double l_pos_x_d = this->get_parameter("pid.left.pos.x.d").as_double();
    double l_pos_y_p = this->get_parameter("pid.left.pos.y.p").as_double();
    double l_pos_y_i = this->get_parameter("pid.left.pos.y.i").as_double();
    double l_pos_y_d = this->get_parameter("pid.left.pos.y.d").as_double();
    double l_pos_z_p = this->get_parameter("pid.left.pos.z.p").as_double();
    double l_pos_z_i = this->get_parameter("pid.left.pos.z.i").as_double();
    double l_pos_z_d = this->get_parameter("pid.left.pos.z.d").as_double();
    double l_pos_deadband = this->get_parameter("pid.left.pos.deadband").as_double();
    
    double l_rot_x_p = this->get_parameter("pid.left.rot.x.p").as_double();
    double l_rot_x_i = this->get_parameter("pid.left.rot.x.i").as_double();
    double l_rot_x_d = this->get_parameter("pid.left.rot.x.d").as_double();
    double l_rot_y_p = this->get_parameter("pid.left.rot.y.p").as_double();
    double l_rot_y_i = this->get_parameter("pid.left.rot.y.i").as_double();
    double l_rot_y_d = this->get_parameter("pid.left.rot.y.d").as_double();
    double l_rot_z_p = this->get_parameter("pid.left.rot.z.p").as_double();
    double l_rot_z_i = this->get_parameter("pid.left.rot.z.i").as_double();
    double l_rot_z_d = this->get_parameter("pid.left.rot.z.d").as_double();
    double l_rot_deadband = this->get_parameter("pid.left.rot.deadband").as_double();

    // Set PID controller gains for the RIGHT arm
    pid_rightarm_ee_pos_x.set_gains(r_pos_x_p, r_pos_x_i, r_pos_x_d, error_sum_tol, decay_rate);
    pid_rightarm_ee_pos_y.set_gains(r_pos_y_p, r_pos_y_i, r_pos_y_d, error_sum_tol, decay_rate);
    pid_rightarm_ee_pos_z.set_gains(r_pos_z_p, r_pos_z_i, r_pos_z_d, error_sum_tol, decay_rate);
    
    pid_rightarm_ee_rot_x.set_gains(r_rot_x_p, r_rot_x_i, r_rot_x_d, error_sum_tol, decay_rate);
    pid_rightarm_ee_rot_y.set_gains(r_rot_y_p, r_rot_y_i, r_rot_y_d, error_sum_tol, decay_rate);
    pid_rightarm_ee_rot_z.set_gains(r_rot_z_p, r_rot_z_i, r_rot_z_d, error_sum_tol, decay_rate);
    
    // Set PID controller gains for the LEFT arm
    pid_leftarm_ee_pos_x.set_gains(l_pos_x_p, l_pos_x_i, l_pos_x_d, error_sum_tol, decay_rate);
    pid_leftarm_ee_pos_y.set_gains(l_pos_y_p, l_pos_y_i, l_pos_y_d, error_sum_tol, decay_rate);
    pid_leftarm_ee_pos_z.set_gains(l_pos_z_p, l_pos_z_i, l_pos_z_d, error_sum_tol, decay_rate);

    pid_leftarm_ee_rot_x.set_gains(l_rot_x_p, l_rot_x_i, l_rot_x_d, error_sum_tol, decay_rate);
    pid_leftarm_ee_rot_y.set_gains(l_rot_y_p, l_rot_y_i, l_rot_y_d, error_sum_tol, decay_rate);
    pid_leftarm_ee_rot_z.set_gains(l_rot_z_p, l_rot_z_i, l_rot_z_d, error_sum_tol, decay_rate);

    RCLCPP_INFO(this->get_logger(), "PID gains have been reloaded.");
}


rcl_interfaces::msg::SetParametersResult EddieRosInterface::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    bool gains_changed = false;

    for (const auto &param : parameters) {
        std::string name = param.get_name();
        
        // Check if any PID parameter was changed
        if (name.rfind("pid.", 0) == 0) {
            gains_changed = true;
            // Activate smoothing based on which arm's gain was changed
            if (name.rfind("pid.right", 0) == 0) {
                RCLCPP_INFO(this->get_logger(), "Right arm PID parameter changed. Activating torque smoothing.");
                this->right_arm_smoothing_start_time_ = std::chrono::steady_clock::now();
                this->right_arm_smoothing_active_.store(true);
            }
            else if (name.rfind("pid.left", 0) == 0) {
                RCLCPP_INFO(this->get_logger(), "Left arm PID parameter changed. Activating torque smoothing.");
                this->left_arm_smoothing_start_time_ = std::chrono::steady_clock::now();
                this->left_arm_smoothing_active_.store(true);
            }
        }
        else if (name == "torque_smoothing_alpha") {
            RCLCPP_INFO(this->get_logger(), "torque_smoothing_alpha parameter changed.");
            // No action here for now
        }
    }

    // If any PID gain was in the list of changed parameters, reload all of them.
    if (gains_changed) {
        this->reload_pid_gains();
    }
    
    return result;
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