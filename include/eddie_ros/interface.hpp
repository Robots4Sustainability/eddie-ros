/*
 * Copyright (c) 2025, SECORO
 *
 * Authors: Vamsi Kalagaturu
 *
 * This file serves as a ROS interface layer for the Eddie robot.
 */

#ifndef EDDIE_ROS_INTERFACE_HPP
#define EDDIE_ROS_INTERFACE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "eddie_ros/action/arm_control.hpp"
#include "eddie_ros/action/gripper_control.hpp"

#include <kdl_parser/kdl_parser.hpp>

#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainjnttojacdotsolver.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

#include "robif2b/functions/ethercat.h"
#include "robif2b/functions/eddie_power_board.h"
#include "robif2b/functions/kelo_drive.h"
#include "robif2b/functions/kinova_gen3.h"

#include "eddie_ros/eddie_ros.fsm.hpp"

#define NUM_DRIVES 4
#define NUM_SLAVES 5
#define NUM_JOINTS 7

#define KINOVA_TAU_CMD_LIMIT 30.0

double evaluate_equality_constraint(double quantity, double reference);
double evaluate_less_than_constraint(double quantity, double threshold);
double evaluate_greater_than_constraint(double quantity, double threshold);
double evaluate_bilateral_constraint(double quantity, double lower, double upper);
void saturate(double *value, double min, double max);

class PID {
  public:
    PID() = default;
    PID(double p_gain,
        double i_gain,
        double d_gain,
        double error_sum_tol = 1.0,
        double decay_rate    = 0.0);

    void set_gains(
        double p_gain,
        double i_gain,
        double d_gain,
        double error_sum_tol = 1.0,
        double decay_rate    = 0.0
    );

    double control(double error, double dt = 1.0);

  public:
    double err_integ;
    double err_last;
    double kp;
    double ki;
    double kd;
    double err_sum_tol;
    double decay_rate;
};

struct EddieState {
    int num_drives;
    struct {
        struct timespec cycle_start;
        struct timespec cycle_end;
        long cycle_time_msr; // [us]
        long cycle_time_exp; // [us]
    } time;
    struct {
        struct robif2b_kelo_drive_api_msr_pdo drv_msr_pdo[NUM_DRIVES];
        struct robif2b_kelo_drive_api_cmd_pdo drv_cmd_pdo[NUM_DRIVES];
        struct robif2b_eddie_power_board_api_msr_pdo pb_msr_pdo;
        struct robif2b_eddie_power_board_api_cmd_pdo pb_cmd_pdo;
    } ecat_comm;
    struct {
        const char *ethernet_if;
        int error_code;
        int num_exposed_slaves;
        int num_found_slaves;
        int num_active_slaves;
        int slave_idx[NUM_SLAVES];
        const char *name[NUM_SLAVES];
        unsigned int prod_code[NUM_SLAVES];
        size_t input_size[NUM_SLAVES];
        size_t output_size[NUM_SLAVES];
        bool is_connected[NUM_SLAVES];
    } ecat;
    struct {
        double pvt_off[NUM_DRIVES];
        double pvt_pos[NUM_DRIVES];
        double pvt_vel[NUM_DRIVES];
        double whl_pos[NUM_DRIVES * 2];
        double whl_vel[NUM_DRIVES * 2];
        double imu_ang_vel[NUM_DRIVES * 3];
        double imu_lin_acc[NUM_DRIVES * 3];
        double bat_volt;
        double bat_cur;
        double bat_pwr;
        uint64_t time_stamp;
        uint16_t status;
    } kelo_msr;
    struct {
        enum robif2b_ctrl_mode ctrl_mode[NUM_DRIVES];
        double vel[NUM_DRIVES * 2];
        double trq[NUM_DRIVES * 2];
        double cur[NUM_DRIVES * 2];
        double max_current[NUM_DRIVES * 2];
        double trq_const[NUM_DRIVES * 2];
    } kelo_cmd;
    struct KinovaArmState {
        bool success;
        // bool gripper_success;
        enum robif2b_ctrl_mode ctrl_mode;
        double pos_msr[NUM_JOINTS];
        double vel_msr[NUM_JOINTS];
        double eff_msr[NUM_JOINTS];
        double cur_msr[NUM_JOINTS];
        double pos_cmd[NUM_JOINTS];
        double vel_cmd[NUM_JOINTS];
        double eff_cmd[NUM_JOINTS];
        double cur_cmd[NUM_JOINTS];
        double imu_ang_vel_msr[3];
        double imu_lin_acc_msr[3];
        // Gripper fields
        float gripper_pos_msr[1];
        float gripper_vel_msr[1];
        float gripper_cur_msr[1];
        float gripper_pos_cmd[1];
        float gripper_vel_cmd[1];
        float gripper_frc_cmd[1];
    };
    KinovaArmState kinova_rightarm_state;
    KinovaArmState kinova_leftarm_state;
};

class EddieRosInterface : public rclcpp::Node {
  public:
    explicit EddieRosInterface(const rclcpp::NodeOptions &options);

    ~EddieRosInterface();

    EddieState eddie_state;

    struct robif2b_ethercat ecat;
    struct robif2b_kelo_drive_encoder drive_enc;
    struct robif2b_kelo_drive_imu imu;
    struct robif2b_kelo_drive_actuator wheel_act;
    struct robif2b_eddie_power_board power_board;
    struct robif2b_kinova_gen3_nbx kinova_rightarm;
    struct robif2b_kinova_gen3_nbx kinova_leftarm;
    struct robif2b_kg3_robotiq_gripper_nbx kinova_rightgripper;
    struct robif2b_kg3_robotiq_gripper_nbx kinova_leftgripper;

    void *input[NUM_SLAVES];
    const void *output[NUM_SLAVES];

    // dynamic parameters
    std::string param_ethernet_if;
    std::string param_arm_select; // "left", "right", or "both"

    // dynamic parametes methods
    rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    void declare_all_parameters();

    void get_all_parameters();

    // Action server
    void initialize_action_servers();

    // sm methods
    void configure(events *eventData, EddieState *eddie_state);
    void idle(events *eventData, const EddieState *eddie_state);
    void compile(events *eventData, const EddieState *eddie_state);
    void execute(events *eventData, EddieState *eddie_state);

    void fsm_behavior(events *eventData, EddieState *eddie_state);

    void compute_gravity_comp(events *eventData, EddieState *eddie_state);
    void compute_cartesian_ctrl(events *eventData, EddieState *eddie_state);

  public:
    void run_fsm();

    bool enable_base_ctrl     = true;
    bool enable_leftarm_ctrl  = true;
    bool enable_rightarm_ctrl = true;

  private:
    KDL::Tree tree;
    KDL::Chain leftarm_chain;
    KDL::Chain rightarm_chain;

    int num_jnts_leftarm;
    int num_segs_leftarm;
    KDL::Twist root_acc_leftarm;
    KDL::JntArray q_leftarm;
    KDL::JntArray qd_leftarm;
    KDL::JntArray qdd_leftarm;
    KDL::JntArray tau_ctrl_leftarm;
    KDL::Wrenches f_ext_leftarm;
    KDL::Frame pose_leftarm_ee;
    KDL::Twist twist_leftarm_ee;
    std::unique_ptr<KDL::ChainIdSolver_RNE> rne_id_solver_leftarm;

    int num_jnts_rightarm;
    int num_segs_rightarm;
    KDL::Twist root_acc_rightarm;
    KDL::JntArray q_rightarm;
    KDL::JntArray qd_rightarm;
    KDL::JntArray qdd_rightarm;
    KDL::JntArray tau_ctrl_rightarm;
    KDL::Wrenches f_ext_rightarm;
    KDL::Frame pose_rightarm_ee;
    KDL::Twist twist_rightarm_ee;
    std::unique_ptr<KDL::ChainIdSolver_RNE> rne_id_solver_rightarm;

    KDL::Frame target_pose_leftarm_ee;
    KDL::Frame target_pose_rightarm_ee;
    KDL::Vector target_pose_wrt_ee;
    KDL::Frame target_pose_offset;

    // Relative target poses from action goals
    KDL::Frame target_pose_leftarm_relative;
    KDL::Frame target_pose_rightarm_relative;
    bool new_target_leftarm = false;
    bool new_target_rightarm = false;

    // Flags to track if arms are currently executing goals
    std::atomic<bool> rightarm_goal_executing = false;
    std::atomic<bool> leftarm_goal_executing = false;
    
    // Flags to track if grippers are currently executing goals
    std::atomic<bool> rightgripper_goal_executing = false;
    std::atomic<bool> leftgripper_goal_executing = false;

    PID pid_leftarm_ee_pos_x;
    PID pid_leftarm_ee_pos_y;
    PID pid_leftarm_ee_pos_z;
    PID pid_leftarm_ee_rot_x;
    PID pid_leftarm_ee_rot_y;
    PID pid_leftarm_ee_rot_z;

    PID pid_rightarm_ee_pos_x;
    PID pid_rightarm_ee_pos_y;
    PID pid_rightarm_ee_pos_z;
    PID pid_rightarm_ee_rot_x;
    PID pid_rightarm_ee_rot_y;
    PID pid_rightarm_ee_rot_z;

    // Helper methods to determine which arms to control
    bool should_control_left_arm() const;
    bool should_control_right_arm() const;

    // Action server callback helper methods
    rclcpp_action::GoalResponse handle_arm_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const eddie_ros::action::ArmControl::Goal> goal,
        const std::string& arm_side);
    
    rclcpp_action::CancelResponse handle_arm_cancel(
        const std::string& arm_side);
    
    void handle_arm_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<eddie_ros::action::ArmControl>> goal_handle,
        const std::string& arm_side);
    
    rclcpp_action::GoalResponse handle_gripper_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const eddie_ros::action::GripperControl::Goal> goal,
        const std::string& arm_side);
    
    rclcpp_action::CancelResponse handle_gripper_cancel(
        const std::string& arm_side);
    
    void handle_gripper_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<eddie_ros::action::GripperControl>> goal_handle,
        const std::string& arm_side);

    // Helper methods for action execution
    void execute_arm_control(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<eddie_ros::action::ArmControl>> goal_handle,
        const std::string& arm_side);
    
    void execute_gripper_control(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<eddie_ros::action::GripperControl>> goal_handle,
        const std::string& arm_side);
    
    // Helper methods to get arm-specific data
    std::atomic<bool>& get_arm_execution_flag(const std::string& arm_side);
    std::atomic<bool>& get_gripper_execution_flag(const std::string& arm_side);
    KDL::Frame& get_target_pose_ee(const std::string& arm_side);
    KDL::Frame& get_current_pose_ee(const std::string& arm_side);
    KDL::Frame& get_target_pose_relative(const std::string& arm_side);
    bool& get_new_target_flag(const std::string& arm_side);
    EddieState::KinovaArmState& get_arm_state(const std::string& arm_side);

    // Action servers
    rclcpp_action::Server<eddie_ros::action::ArmControl>::SharedPtr action_server_right_arm_control_;
    rclcpp_action::Server<eddie_ros::action::GripperControl>::SharedPtr action_server_right_gripper_control_;
    rclcpp_action::Server<eddie_ros::action::ArmControl>::SharedPtr action_server_left_arm_control_;
    rclcpp_action::Server<eddie_ros::action::GripperControl>::SharedPtr action_server_left_gripper_control_;
};

#endif // EDDIE_ROS_INTERFACE_HPP