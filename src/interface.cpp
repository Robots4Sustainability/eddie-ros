/*
 * Copyright (c) 2025, SECORO
 *
 * Authors: Vamsi Kalagaturu
 */

#include "eddie-ros/interface.hpp"
#include <signal.h>
#include <time.h>

volatile sig_atomic_t keep_running = 1;

static long timespec_to_usec(const struct timespec *t) {
    const int NSEC_IN_USEC = 1000;
    const int USEC_IN_SEC  = 1000000;

    return t->tv_sec * USEC_IN_SEC + t->tv_nsec / NSEC_IN_USEC;
}

void sigint_handler(int signum) { keep_running = 0; }

double evaluate_equality_constraint(double quantity, double reference) {
    return quantity - reference;
}

double evaluate_less_than_constraint(double quantity, double threshold) {
    return (quantity < threshold) ? 0.0 : threshold - quantity;
}

double evaluate_greater_than_constraint(double quantity, double threshold) {
    return (quantity > threshold) ? 0.0 : quantity - threshold;
}

double evaluate_bilateral_constraint(double quantity, double lower, double upper) {
    if (quantity < lower)
        return lower - quantity;
    else if (quantity > upper)
        return quantity - upper;
    else
        return 0.0;
}

void saturate(double *value, double min, double max) {
    if (*value < min) {
        *value = min;
    } else if (*value > max) {
        *value = max;
    }
}

PID::PID(double p_gain, double i_gain, double d_gain, double error_sum_tol, double decay_rate) {
    err_integ        = 0.0;
    err_last         = 0.0;
    kp               = p_gain;
    ki               = i_gain;
    kd               = d_gain;
    err_sum_tol      = error_sum_tol;
    this->decay_rate = decay_rate;
}

void PID::set_gains(
    double p_gain, double i_gain, double d_gain, double error_sum_tol, double decay_rate
) {
    err_integ        = 0.0;
    err_last         = 0.0;
    kp               = p_gain;
    ki               = i_gain;
    kd               = d_gain;
    err_sum_tol      = error_sum_tol;
    this->decay_rate = decay_rate;
}

double PID::control(double error, double dt) {
    double err_diff = (error - err_last) / dt;

    if (fabs(error) > 0.0) {
        // Accumulate the integral when error is non-zero
        err_integ += error * dt;

        // Clamp the integral term to prevent runaway accumulation
        if (err_integ > err_sum_tol) {
            err_integ = err_sum_tol;
        } else if (err_integ < -err_sum_tol) {
            err_integ = -err_sum_tol;
        }
    } else {
        // Decay the integral term when the error is zero
        err_integ = decay_rate * err_integ + (1.0 - decay_rate) * error;
    }

    // err_integ = decay_rate * err_integ + (1.0 - decay_rate) * error;
    err_last = error;

    return kp * error + ki * err_integ + kd * err_diff;
}

EddieRosInterface::EddieRosInterface(const rclcpp::NodeOptions &options)
    : rclcpp::Node("eddie_ros_interface", options) {

    signal(SIGINT, sigint_handler);

    // Declare parameters
    this->declare_all_parameters();

    eddie_state     = {};
    ecat            = {};
    drive_enc       = {};
    imu             = {};
    wheel_act       = {};
    power_board     = {};
    kinova_rightarm = {};
    kinova_leftarm  = {};

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("eddie-ros");
    std::string urdf_path               = package_share_directory + "/urdf/eddie.urdf";

    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
        RCLCPP_ERROR(get_logger(), "Failed to construct kdl tree");
        exit(11);
    } else {
        RCLCPP_INFO(get_logger(), "KDL tree constructed successfully");
    }
    if (!tree.getChain("base_link", "kinova_left_grasp_link", leftarm_chain)) {
        RCLCPP_ERROR(get_logger(), "Failed to get left arm chain");
        exit(11);
    } else {
        RCLCPP_INFO(get_logger(), "Left arm chain constructed successfully");
    }
    if (!tree.getChain("base_link", "kinova_right_grasp_link", rightarm_chain)) {
        RCLCPP_ERROR(get_logger(), "Failed to get right arm chain");
        exit(11);
    } else {
        RCLCPP_INFO(get_logger(), "Right arm chain constructed successfully");
    }

    // joint inertias:
    const std::vector<double> joint_inertia{0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};

    // set joint inertias
    for (size_t i = 0; i < rightarm_chain.getNrOfJoints(); i++) {
        rightarm_chain.getSegment(i).getMutableJoint().setInertia(joint_inertia[i]);
    }
    for (size_t i = 0; i < leftarm_chain.getNrOfJoints(); i++) {
        leftarm_chain.getSegment(i).getMutableJoint().setInertia(joint_inertia[i]);
    }

    num_jnts_leftarm = leftarm_chain.getNrOfJoints();
    num_segs_leftarm = leftarm_chain.getNrOfSegments();
    // root_acc_leftarm = KDL::Twist(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector::Zero());
    root_acc_leftarm = KDL::Twist(KDL::Vector(0.0, 0.0, -9.81), KDL::Vector::Zero());
    q_leftarm.resize(num_jnts_leftarm);
    qd_leftarm.resize(num_jnts_leftarm);
    qdd_leftarm.resize(num_jnts_leftarm);
    tau_ctrl_leftarm.resize(num_jnts_leftarm);
    f_ext_leftarm.resize(num_segs_leftarm);
    rne_id_solver_leftarm =
        std::make_unique<KDL::ChainIdSolver_RNE>(leftarm_chain, root_acc_leftarm.vel);

    num_jnts_rightarm = rightarm_chain.getNrOfJoints();
    num_segs_rightarm = rightarm_chain.getNrOfSegments();
    // root_acc_rightarm = KDL::Twist(KDL::Vector(-9.473, -1.02, 1.142), KDL::Vector::Zero());
    root_acc_rightarm = KDL::Twist(KDL::Vector(0.0, 0.0, -9.81), KDL::Vector::Zero());
    q_rightarm.resize(num_jnts_rightarm);
    qd_rightarm.resize(num_jnts_rightarm);
    qdd_rightarm.resize(num_jnts_rightarm);
    tau_ctrl_rightarm.resize(num_jnts_rightarm);
    f_ext_rightarm.resize(num_segs_rightarm);
    rne_id_solver_rightarm =
        std::make_unique<KDL::ChainIdSolver_RNE>(rightarm_chain, root_acc_rightarm.vel);

    // PID controller gains
    pid_rightarm_ee_pos_x.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_pos_y.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_pos_z.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_rot_x.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_rot_y.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_rot_z.set_gains(50.0, 0., 0.0, 0.9);
    
    pid_leftarm_ee_pos_x.set_gains(50.0, 0., 0.0, 0.9);
    pid_leftarm_ee_pos_y.set_gains(50.0, 0., 0.0, 0.9);
    pid_leftarm_ee_pos_z.set_gains(50.0, 0., 0.0, 0.9);
    pid_leftarm_ee_rot_x.set_gains(50.0, 0., 0.0, 0.9);
    pid_leftarm_ee_rot_y.set_gains(50.0, 0., 0.0, 0.9);
    pid_leftarm_ee_rot_z.set_gains(50.0, 0., 0.0, 0.9);

    RCLCPP_INFO(get_logger(), "Eddie ROS interface node initialized.");
}

EddieRosInterface::~EddieRosInterface() {
    // robif2b_kelo_drive_actuator_stop(&wheel_act);
    // robif2b_ethercat_stop(&ecat);
    // robif2b_ethercat_shutdown(&ecat);
    // if (eddie_state.ecat.error_code < 0) {
    //     RCLCPP_ERROR(get_logger(), "EtherCAT stop failed.");
    //     return;
    // }

    if (param_arm_to_control == "rightarm") {
        robif2b_kinova_gen3_stop(&kinova_rightarm);
        robif2b_kinova_gen3_shutdown(&kinova_rightarm);
    } else if (param_arm_to_control == "leftarm") {
        robif2b_kinova_gen3_stop(&kinova_leftarm);
        robif2b_kinova_gen3_shutdown(&kinova_leftarm);
    }
}

void EddieRosInterface::configure(events *eventData, EddieState *eddie_state) {
    eddie_state->num_drives              = NUM_DRIVES;
    eddie_state->time.cycle_time_exp     = 1000; // [us]
    eddie_state->ecat.ethernet_if        = param_ethernet_if.c_str();
    eddie_state->ecat.num_exposed_slaves = NUM_SLAVES;
    eddie_state->ecat.slave_idx[0]       = 1; // power board
    eddie_state->ecat.slave_idx[1]       = 3; // drive 1 - Front Left
    eddie_state->ecat.slave_idx[2]       = 4; // drive 2 - Rear Left
    eddie_state->ecat.slave_idx[3]       = 6; // drive 3 - Rear Right
    eddie_state->ecat.slave_idx[4]       = 7; // drive 4 - Front Right

    for (int i = 0; i < NUM_DRIVES; i++) {
        eddie_state->kelo_cmd.ctrl_mode[i]           = ROBIF2B_CTRL_MODE_FORCE;
        eddie_state->kelo_cmd.max_current[i * 2 + 0] = 10;   // [A]
        eddie_state->kelo_cmd.max_current[i * 2 + 1] = 10;   // [A]
        eddie_state->kelo_cmd.trq_const[i * 2 + 0]   = 0.29; // [Nm/A]
        eddie_state->kelo_cmd.trq_const[i * 2 + 1]   = 0.29; // [Nm/A]
    }
    eddie_state->kelo_msr.pvt_off[0] = 0.0;
    eddie_state->kelo_msr.pvt_off[1] = 0.0;
    eddie_state->kelo_msr.pvt_off[2] = 0.0;
    eddie_state->kelo_msr.pvt_off[3] = 0.0;

    eddie_state->ecat.name[0]        = "KELO_ECAT_FRD2_PMU1.0";
    eddie_state->ecat.prod_code[0]   = 0x90001001;
    eddie_state->ecat.input_size[0]  = sizeof(eddie_state->ecat_comm.pb_msr_pdo);
    eddie_state->ecat.output_size[0] = sizeof(eddie_state->ecat_comm.pb_cmd_pdo);
    for (int i = 1; i < NUM_DRIVES + 1; i++) {
        eddie_state->ecat.name[i]        = "KELOD105";
        eddie_state->ecat.prod_code[i]   = 0x02001001;
        eddie_state->ecat.input_size[i]  = sizeof(eddie_state->ecat_comm.drv_msr_pdo[i - 1]);
        eddie_state->ecat.output_size[i] = sizeof(eddie_state->ecat_comm.drv_cmd_pdo[i - 1]);
    }

    // Set only the selected arm's state to active, zero the other
    if (param_arm_to_control == "rightarm") {
        eddie_state->kinova_rightarm_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
        eddie_state->kinova_rightarm_state.success   = false;
        for (int i = 0; i < NUM_JOINTS; i++) {
            eddie_state->kinova_rightarm_state.pos_msr[i] = 0.0;
            eddie_state->kinova_rightarm_state.vel_msr[i] = 0.0;
            eddie_state->kinova_rightarm_state.eff_msr[i] = 0.0;
            eddie_state->kinova_rightarm_state.cur_msr[i] = 0.0;
            eddie_state->kinova_rightarm_state.pos_cmd[i] = 0.0;
            eddie_state->kinova_rightarm_state.vel_cmd[i] = 0.0;
            eddie_state->kinova_rightarm_state.eff_cmd[i] = 0.0;
            eddie_state->kinova_rightarm_state.cur_cmd[i] = 0.0;
        }
        eddie_state->kinova_rightarm_state.imu_ang_vel_msr[0] = 0.0;
        eddie_state->kinova_rightarm_state.imu_ang_vel_msr[1] = 0.0;
        eddie_state->kinova_rightarm_state.imu_ang_vel_msr[2] = 0.0;
        eddie_state->kinova_rightarm_state.imu_lin_acc_msr[0] = 0.0;
        eddie_state->kinova_rightarm_state.imu_lin_acc_msr[1] = 0.0;
        eddie_state->kinova_rightarm_state.imu_lin_acc_msr[2] = 0.0;
        // Set default gripper command values for right arm
        eddie_state->kinova_rightarm_state.gripper_pos_cmd[0] = 0.0;
        eddie_state->kinova_rightarm_state.gripper_vel_cmd[0] = 0.0;
        eddie_state->kinova_rightarm_state.gripper_frc_cmd[0] = 0.0;
    } else if (param_arm_to_control == "leftarm") {
        eddie_state->kinova_leftarm_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
        eddie_state->kinova_leftarm_state.success   = false;
        for (int i = 0; i < NUM_JOINTS; i++) {
            eddie_state->kinova_leftarm_state.pos_msr[i] = 0.0;
            eddie_state->kinova_leftarm_state.vel_msr[i] = 0.0;
            eddie_state->kinova_leftarm_state.eff_msr[i] = 0.0;
            eddie_state->kinova_leftarm_state.cur_msr[i] = 0.0;
            eddie_state->kinova_leftarm_state.pos_cmd[i] = 0.0;
            eddie_state->kinova_leftarm_state.vel_cmd[i] = 0.0;
            eddie_state->kinova_leftarm_state.eff_cmd[i] = 0.0;
            eddie_state->kinova_leftarm_state.cur_cmd[i] = 0.0;
        }
        eddie_state->kinova_leftarm_state.imu_ang_vel_msr[0] = 0.0;
        eddie_state->kinova_leftarm_state.imu_ang_vel_msr[1] = 0.0;
        eddie_state->kinova_leftarm_state.imu_ang_vel_msr[2] = 0.0;
        eddie_state->kinova_leftarm_state.imu_lin_acc_msr[0] = 0.0;
        eddie_state->kinova_leftarm_state.imu_lin_acc_msr[1] = 0.0;
        eddie_state->kinova_leftarm_state.imu_lin_acc_msr[2] = 0.0;
        // Set default gripper command values for left arm
        eddie_state->kinova_leftarm_state.gripper_pos_cmd[0] = 0.0;
        eddie_state->kinova_leftarm_state.gripper_vel_cmd[0] = 0.0;
        eddie_state->kinova_leftarm_state.gripper_frc_cmd[0] = 0.0;
    }


    // Connections
    ecat.ethernet_if        = &eddie_state->ecat.ethernet_if[0];
    ecat.num_exposed_slaves = &eddie_state->ecat.num_exposed_slaves;
    ecat.slave_idx          = &eddie_state->ecat.slave_idx[0];
    ecat.name               = &eddie_state->ecat.name[0];
    ecat.product_code       = &eddie_state->ecat.prod_code[0];
    ecat.input_size         = &eddie_state->ecat.input_size[0];
    ecat.output_size        = &eddie_state->ecat.output_size[0];
    ecat.error_code         = &eddie_state->ecat.error_code;
    ecat.num_initial_slaves = &eddie_state->ecat.num_found_slaves;
    ecat.num_current_slaves = &eddie_state->ecat.num_active_slaves;
    ecat.is_connected       = &eddie_state->ecat.is_connected[0];

    input[0] = &eddie_state->ecat_comm.pb_msr_pdo;
    input[1] = &eddie_state->ecat_comm.drv_msr_pdo[0];
    input[2] = &eddie_state->ecat_comm.drv_msr_pdo[1];
    input[3] = &eddie_state->ecat_comm.drv_msr_pdo[2];
    input[4] = &eddie_state->ecat_comm.drv_msr_pdo[3];

    output[0] = &eddie_state->ecat_comm.pb_cmd_pdo;
    output[1] = &eddie_state->ecat_comm.drv_cmd_pdo[0];
    output[2] = &eddie_state->ecat_comm.drv_cmd_pdo[1];
    output[3] = &eddie_state->ecat_comm.drv_cmd_pdo[2];
    output[4] = &eddie_state->ecat_comm.drv_cmd_pdo[3];

    ecat.input  = input;
    ecat.output = output;

    drive_enc.num_drives    = &eddie_state->num_drives;
    drive_enc.msr_pdo       = &eddie_state->ecat_comm.drv_msr_pdo[0];
    drive_enc.wheel_pos_msr = &eddie_state->kelo_msr.whl_pos[0];
    drive_enc.wheel_vel_msr = &eddie_state->kelo_msr.whl_vel[0];
    drive_enc.pivot_pos_msr = &eddie_state->kelo_msr.pvt_pos[0];
    drive_enc.pivot_vel_msr = &eddie_state->kelo_msr.pvt_vel[0];
    drive_enc.pivot_pos_off = &eddie_state->kelo_msr.pvt_off[0];

    imu.num_drives      = &eddie_state->num_drives;
    imu.msr_pdo         = &eddie_state->ecat_comm.drv_msr_pdo[0];
    imu.imu_ang_vel_msr = &eddie_state->kelo_msr.imu_ang_vel[0];
    imu.imu_lin_acc_msr = &eddie_state->kelo_msr.imu_lin_acc[0];

    wheel_act.num_drives  = &eddie_state->num_drives;
    wheel_act.cmd_pdo     = &eddie_state->ecat_comm.drv_cmd_pdo[0];
    wheel_act.ctrl_mode   = &eddie_state->kelo_cmd.ctrl_mode[0];
    wheel_act.act_vel_cmd = &eddie_state->kelo_cmd.vel[0];
    wheel_act.act_trq_cmd = &eddie_state->kelo_cmd.trq[0];
    wheel_act.act_cur_cmd = &eddie_state->kelo_cmd.cur[0];
    wheel_act.max_current = &eddie_state->kelo_cmd.max_current[0];
    wheel_act.trq_const   = &eddie_state->kelo_cmd.trq_const[0];

    power_board.msr_pdo     = &eddie_state->ecat_comm.pb_msr_pdo;
    power_board.cmd_pdo     = &eddie_state->ecat_comm.pb_cmd_pdo;
    power_board.time_stamp  = &eddie_state->kelo_msr.time_stamp;
    power_board.status      = &eddie_state->kelo_msr.status;
    power_board.voltage_msr = &eddie_state->kelo_msr.bat_volt;
    power_board.current_msr = &eddie_state->kelo_msr.bat_cur;
    power_board.power_msr   = &eddie_state->kelo_msr.bat_pwr;

    double cycle_time                       = 0.001;

    kinova_rightarm.conf.ip_address         = "192.168.1.12";
    kinova_rightarm.conf.port               = 10000;
    kinova_rightarm.conf.port_real_time     = 10001;
    kinova_rightarm.conf.user               = "admin";
    kinova_rightarm.conf.password           = "admin";
    kinova_rightarm.conf.session_timeout    = 60000;
    kinova_rightarm.conf.connection_timeout = 2000;
    // Enable gripper for right arm
    kinova_rightarm.conf.use_gripper        = true;
    kinova_rightarm.cycle_time              = &cycle_time;
    kinova_rightarm.ctrl_mode               = &eddie_state->kinova_rightarm_state.ctrl_mode;
    kinova_rightarm.jnt_pos_msr             = &eddie_state->kinova_rightarm_state.pos_msr[0];
    kinova_rightarm.jnt_vel_msr             = &eddie_state->kinova_rightarm_state.vel_msr[0];
    kinova_rightarm.jnt_trq_msr             = &eddie_state->kinova_rightarm_state.eff_msr[0];
    kinova_rightarm.act_cur_msr             = &eddie_state->kinova_rightarm_state.cur_msr[0];
    kinova_rightarm.jnt_pos_cmd             = &eddie_state->kinova_rightarm_state.pos_cmd[0];
    kinova_rightarm.jnt_vel_cmd             = &eddie_state->kinova_rightarm_state.vel_cmd[0];
    kinova_rightarm.jnt_trq_cmd             = &eddie_state->kinova_rightarm_state.eff_cmd[0];
    kinova_rightarm.act_cur_cmd             = &eddie_state->kinova_rightarm_state.cur_cmd[0];
    kinova_rightarm.success                 = &eddie_state->kinova_rightarm_state.success;
    kinova_rightarm.imu_ang_vel_msr         = &eddie_state->kinova_rightarm_state.imu_ang_vel_msr[0];
    kinova_rightarm.imu_lin_acc_msr         = &eddie_state->kinova_rightarm_state.imu_lin_acc_msr[0];
    // Gripper connections for right arm
    kinova_rightarm.gripper_pos_msr         = &eddie_state->kinova_rightarm_state.gripper_pos_msr[0];
    kinova_rightarm.gripper_vel_msr         = &eddie_state->kinova_rightarm_state.gripper_vel_msr[0];
    kinova_rightarm.gripper_cur_msr         = &eddie_state->kinova_rightarm_state.gripper_cur_msr[0];
    kinova_rightarm.gripper_pos_cmd         = &eddie_state->kinova_rightarm_state.gripper_pos_cmd[0];
    kinova_rightarm.gripper_vel_cmd         = &eddie_state->kinova_rightarm_state.gripper_vel_cmd[0];
    kinova_rightarm.gripper_frc_cmd         = &eddie_state->kinova_rightarm_state.gripper_frc_cmd[0];
    
    kinova_leftarm.conf.ip_address         = "192.168.1.10";
    kinova_leftarm.conf.port               = 10000;
    kinova_leftarm.conf.port_real_time     = 10001;
    kinova_leftarm.conf.user               = "admin";
    kinova_leftarm.conf.password           = "admin";
    kinova_leftarm.conf.session_timeout    = 60000;
    kinova_leftarm.conf.connection_timeout = 2000;
    // Enable gripper for left arm
    kinova_leftarm.conf.use_gripper        = true;
    kinova_leftarm.cycle_time              = &cycle_time;
    kinova_leftarm.ctrl_mode               = &eddie_state->kinova_leftarm_state.ctrl_mode;
    kinova_leftarm.jnt_pos_msr             = &eddie_state->kinova_leftarm_state.pos_msr[0];
    kinova_leftarm.jnt_vel_msr             = &eddie_state->kinova_leftarm_state.vel_msr[0];
    kinova_leftarm.jnt_trq_msr             = &eddie_state->kinova_leftarm_state.eff_msr[0];
    kinova_leftarm.act_cur_msr             = &eddie_state->kinova_leftarm_state.cur_msr[0];
    kinova_leftarm.jnt_pos_cmd             = &eddie_state->kinova_leftarm_state.pos_cmd[0];
    kinova_leftarm.jnt_vel_cmd             = &eddie_state->kinova_leftarm_state.vel_cmd[0];
    kinova_leftarm.jnt_trq_cmd             = &eddie_state->kinova_leftarm_state.eff_cmd[0];
    kinova_leftarm.act_cur_cmd             = &eddie_state->kinova_leftarm_state.cur_cmd[0];
    kinova_leftarm.success                 = &eddie_state->kinova_leftarm_state.success;
    kinova_leftarm.imu_ang_vel_msr         = &eddie_state->kinova_leftarm_state.imu_ang_vel_msr[0];
    kinova_leftarm.imu_lin_acc_msr         = &eddie_state->kinova_leftarm_state.imu_lin_acc_msr[0];
    // Gripper connections for left arm
    kinova_leftarm.gripper_pos_msr         = &eddie_state->kinova_leftarm_state.gripper_pos_msr[0];
    kinova_leftarm.gripper_vel_msr         = &eddie_state->kinova_leftarm_state.gripper_vel_msr[0];
    kinova_leftarm.gripper_cur_msr         = &eddie_state->kinova_leftarm_state.gripper_cur_msr[0];
    kinova_leftarm.gripper_pos_cmd         = &eddie_state->kinova_leftarm_state.gripper_pos_cmd[0];
    kinova_leftarm.gripper_vel_cmd         = &eddie_state->kinova_leftarm_state.gripper_vel_cmd[0];
    kinova_leftarm.gripper_frc_cmd         = &eddie_state->kinova_leftarm_state.gripper_frc_cmd[0];

    // RCLCPP_INFO(get_logger(), "ethercat_if: %s", eddie_state->ecat.ethernet_if);

    // robif2b_ethercat_configure(&ecat);
    // if (eddie_state->ecat.error_code < 0) {
    //     RCLCPP_ERROR(get_logger(), "EtherCAT configuration failed.");
    //     return;
    // }

    // robif2b_ethercat_start(&ecat);
    // if (eddie_state->ecat.error_code < 0) {
    //     RCLCPP_ERROR(get_logger(), "EtherCAT start failed.");
    //     return;
    // }

    // power_board.cmd_pdo->shutdown = 0;
    // power_board.cmd_pdo->command  = 0b00100000;
    // robif2b_eddie_power_board_update(&power_board);

    // robif2b_ethercat_update(&ecat);
    // if (eddie_state->ecat.error_code < 0) {
    //     RCLCPP_ERROR(get_logger(), "EtherCAT update failed.");
    //     return;
    // }

    // kinova
    if (param_arm_to_control == "rightarm") {
        RCLCPP_INFO(get_logger(), "Configuring right arm");
        robif2b_kinova_gen3_configure(&kinova_rightarm);
        robif2b_kinova_gen3_recover(&kinova_rightarm);
        robif2b_kinova_gen3_start(&kinova_rightarm);
    } else if (param_arm_to_control == "leftarm") {
        RCLCPP_INFO(get_logger(), "Configuring left arm");
        robif2b_kinova_gen3_configure(&kinova_leftarm);
        robif2b_kinova_gen3_recover(&kinova_leftarm);
        robif2b_kinova_gen3_start(&kinova_leftarm);
    }

    RCLCPP_INFO(get_logger(), "Eddie ROS interface configured.");

    RCLCPP_DEBUG(get_logger(), "In configure state");
    produce_event(eventData, E_CONFIGURE_EXIT);
}

void EddieRosInterface::idle(events *eventData, const EddieState *eddie_state) {
    if (param_arm_to_control == "rightarm") {
        robif2b_kinova_gen3_update(&kinova_rightarm);
        for (int i = 0; i < num_jnts_rightarm; i++) {
            q_rightarm(i)  = eddie_state->kinova_rightarm_state.pos_msr[i];
            qd_rightarm(i) = eddie_state->kinova_rightarm_state.vel_msr[i];
        }
        KDL::JntArrayVel q_qd_rightarm(q_rightarm, qd_rightarm);
        KDL::ChainFkSolverPos_recursive fpk_pose_rightarm_ee(rightarm_chain);
        fpk_pose_rightarm_ee.JntToCart(q_rightarm, pose_rightarm_ee);
        KDL::ChainFkSolverVel_recursive fvk_twist_rightarm_ee(rightarm_chain);
        KDL::FrameVel _twist_rightarm_ee;
        fvk_twist_rightarm_ee.JntToCart(q_qd_rightarm, _twist_rightarm_ee);
        twist_rightarm_ee = _twist_rightarm_ee.deriv();
        target_pose_rightarm_ee = pose_rightarm_ee;
    } else if (param_arm_to_control == "leftarm") {
        robif2b_kinova_gen3_update(&kinova_leftarm);
        for (int i = 0; i < num_jnts_leftarm; i++) {
            q_leftarm(i)  = eddie_state->kinova_leftarm_state.pos_msr[i];
            qd_leftarm(i) = eddie_state->kinova_leftarm_state.vel_msr[i];
        }
        KDL::JntArrayVel q_qd_leftarm(q_leftarm, qd_leftarm);
        KDL::ChainFkSolverPos_recursive fpk_pose_leftarm_ee(leftarm_chain);
        fpk_pose_leftarm_ee.JntToCart(q_leftarm, pose_leftarm_ee);
        KDL::ChainFkSolverVel_recursive fvk_twist_leftarm_ee(leftarm_chain);
        KDL::FrameVel _twist_leftarm_ee;
        fvk_twist_leftarm_ee.JntToCart(q_qd_leftarm, _twist_leftarm_ee);
        twist_leftarm_ee = _twist_leftarm_ee.deriv();
        target_pose_leftarm_ee = pose_leftarm_ee;
    }
    RCLCPP_DEBUG(get_logger(), "Exiting idle state");
    produce_event(eventData, E_IDLE_EXIT_EXECUTE);
}

void EddieRosInterface::compile(events *eventData, const EddieState *eddie_state) {
    RCLCPP_DEBUG(get_logger(), "Exiting compile state");
    produce_event(eventData, E_COMPILE_EXIT);
}

void EddieRosInterface::compute_gravity_comp(events *eventData, EddieState *eddie_state) {
    if (param_arm_to_control == "rightarm") {
        for (auto &wrench : f_ext_rightarm) {
            wrench = KDL::Wrench::Zero();
        }
        int r = 0;
        KDL::JntArrayVel jnt_array_vel_rightarm(q_rightarm, qd_rightarm);
        KDL::Twist jd_qd_rightarm;
        KDL::Twist xdd_minus_jd_qd_rightarm;
        KDL::Twist xdd;
        KDL::ChainJntToJacDotSolver jnt_to_jac_dot_solver_rightarm(rightarm_chain);
        KDL::ChainIkSolverVel_pinv ik_solver_vel_rightarm(rightarm_chain);
        jnt_to_jac_dot_solver_rightarm.JntToJacDot(jnt_array_vel_rightarm, jd_qd_rightarm);
        xdd_minus_jd_qd_rightarm = xdd - jd_qd_rightarm;
        ik_solver_vel_rightarm.CartToJnt(q_rightarm, xdd_minus_jd_qd_rightarm, qdd_rightarm);
        r = rne_id_solver_rightarm->CartToJnt(
            q_rightarm, qd_rightarm, qdd_rightarm, f_ext_rightarm, tau_ctrl_rightarm
        );
        if (r < 0) {
            RCLCPP_ERROR(get_logger(), "Right arm RNE ID solver failed with error code: %d", r);
            return;
        }
        for (int i = 0; i < num_jnts_rightarm; i++) {
            saturate(&tau_ctrl_rightarm(i), -KINOVA_TAU_CMD_LIMIT, KINOVA_TAU_CMD_LIMIT);
            eddie_state->kinova_rightarm_state.eff_cmd[i] = tau_ctrl_rightarm(i);
        }
    } else if (param_arm_to_control == "leftarm") {
        for (auto &wrench : f_ext_leftarm) {
            wrench = KDL::Wrench::Zero();
        }
        int r = 0;
        KDL::JntArrayVel jnt_array_vel_leftarm(q_leftarm, qd_leftarm);
        KDL::Twist jd_qd_leftarm;
        KDL::Twist xdd_minus_jd_qd_leftarm;
        KDL::Twist xdd_left;
        KDL::ChainJntToJacDotSolver jnt_to_jac_dot_solver_leftarm(leftarm_chain);
        KDL::ChainIkSolverVel_pinv ik_solver_vel_leftarm(leftarm_chain);
        jnt_to_jac_dot_solver_leftarm.JntToJacDot(jnt_array_vel_leftarm, jd_qd_leftarm);
        xdd_minus_jd_qd_leftarm = xdd_left - jd_qd_leftarm;
        ik_solver_vel_leftarm.CartToJnt(q_leftarm, xdd_minus_jd_qd_leftarm, qdd_leftarm);
        r = rne_id_solver_leftarm->CartToJnt(
            q_leftarm, qd_leftarm, qdd_leftarm, f_ext_leftarm, tau_ctrl_leftarm
        );
        if (r < 0) {
            RCLCPP_ERROR(get_logger(), "Left arm RNE ID solver failed with error code: %d", r);
            return;
        }
        for (int i = 0; i < num_jnts_leftarm; i++) {
            saturate(&tau_ctrl_leftarm(i), -KINOVA_TAU_CMD_LIMIT, KINOVA_TAU_CMD_LIMIT);
            eddie_state->kinova_leftarm_state.eff_cmd[i] = tau_ctrl_leftarm(i);
        }
    }
}

void EddieRosInterface::compute_cartesian_ctrl(events *eventData, EddieState *eddie_state) {

    long cycle_time_msr = eddie_state->time.cycle_time_msr;

    // convert to seconds
    double cycle_time = static_cast<double>(cycle_time_msr) / 1e6;
    if (cycle_time <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Invalid cycle time: %ld", cycle_time_msr);
        return;
    }
    if (param_arm_to_control == "rightarm") {
        KDL::Twist delta_pose_rightarm_ee = KDL::diff(target_pose_rightarm_ee, pose_rightarm_ee);

        double fx = pid_rightarm_ee_pos_x.control(delta_pose_rightarm_ee.vel.x(), cycle_time);
        double fy = pid_rightarm_ee_pos_y.control(delta_pose_rightarm_ee.vel.y(), cycle_time);
        double fz = pid_rightarm_ee_pos_z.control(delta_pose_rightarm_ee.vel.z(), cycle_time);
        double mx = pid_rightarm_ee_rot_x.control(delta_pose_rightarm_ee.rot.x(), cycle_time);
        double my = pid_rightarm_ee_rot_y.control(delta_pose_rightarm_ee.rot.y(), cycle_time);
        double mz = pid_rightarm_ee_rot_z.control(delta_pose_rightarm_ee.rot.z(), cycle_time);

        KDL::Wrench f_ext_ee_rightarm = KDL::Wrench(KDL::Vector(fx, fy, fz), KDL::Vector(mx, my, mz));
        KDL::Wrench f_ext_ee_rightarm_wrt_ee = KDL::Wrench(
            pose_rightarm_ee.M.Inverse() * f_ext_ee_rightarm.force,
            pose_rightarm_ee.M.Inverse() * f_ext_ee_rightarm.torque
        );

        for (auto &wrench : f_ext_rightarm) {
            wrench = KDL::Wrench::Zero();
        }
        f_ext_rightarm[num_segs_rightarm - 1] = f_ext_ee_rightarm_wrt_ee;

        KDL::JntArrayVel jnt_array_vel_rightarm(q_rightarm, qd_rightarm);
        KDL::Twist jd_qd_rightarm;
        KDL::Twist xdd_minus_jd_qd_rightarm;
        KDL::Twist xdd;

        KDL::ChainJntToJacDotSolver jnt_to_jac_dot_solver_rightarm(rightarm_chain);
        KDL::ChainIkSolverVel_pinv ik_solver_vel_rightarm(rightarm_chain);
        jnt_to_jac_dot_solver_rightarm.JntToJacDot(jnt_array_vel_rightarm, jd_qd_rightarm);
        xdd_minus_jd_qd_rightarm = xdd - jd_qd_rightarm;
        ik_solver_vel_rightarm.CartToJnt(q_rightarm, xdd_minus_jd_qd_rightarm, qdd_rightarm);

        int r = rne_id_solver_rightarm->CartToJnt(
            q_rightarm, qd_rightarm, qdd_rightarm, f_ext_rightarm, tau_ctrl_rightarm
        );
        if (r < 0) {
            RCLCPP_ERROR(get_logger(), "Right arm RNE ID solver failed with error code: %d", r);
        }
        for (int i = 0; i < num_jnts_rightarm; i++) {
            saturate(&tau_ctrl_rightarm(i), -KINOVA_TAU_CMD_LIMIT, KINOVA_TAU_CMD_LIMIT);
            eddie_state->kinova_rightarm_state.eff_cmd[i] = tau_ctrl_rightarm(i);
        }
    } else if (param_arm_to_control == "leftarm") {
        KDL::Twist delta_pose_leftarm_ee = KDL::diff(target_pose_leftarm_ee, pose_leftarm_ee);

        double fx_left = pid_leftarm_ee_pos_x.control(delta_pose_leftarm_ee.vel.x(), cycle_time);
        double fy_left = pid_leftarm_ee_pos_y.control(delta_pose_leftarm_ee.vel.y(), cycle_time);
        double fz_left = pid_leftarm_ee_pos_z.control(delta_pose_leftarm_ee.vel.z(), cycle_time);
        double mx_left = pid_leftarm_ee_rot_x.control(delta_pose_leftarm_ee.rot.x(), cycle_time);
        double my_left = pid_leftarm_ee_rot_y.control(delta_pose_leftarm_ee.rot.y(), cycle_time);
        double mz_left = pid_leftarm_ee_rot_z.control(delta_pose_leftarm_ee.rot.z(), cycle_time);

        KDL::Wrench f_ext_ee_leftarm = KDL::Wrench(KDL::Vector(fx_left, fy_left, fz_left), KDL::Vector(mx_left, my_left, mz_left));
        KDL::Wrench f_ext_ee_leftarm_wrt_ee = KDL::Wrench(
            pose_leftarm_ee.M.Inverse() * f_ext_ee_leftarm.force,
            pose_leftarm_ee.M.Inverse() * f_ext_ee_leftarm.torque
        );

        for (auto &wrench : f_ext_leftarm) {
            wrench = KDL::Wrench::Zero();
        }
        f_ext_leftarm[num_segs_leftarm - 1] = f_ext_ee_leftarm_wrt_ee;

        KDL::JntArrayVel jnt_array_vel_leftarm(q_leftarm, qd_leftarm);
        KDL::Twist jd_qd_leftarm;
        KDL::Twist xdd_minus_jd_qd_leftarm;
        KDL::Twist xdd;

        KDL::ChainJntToJacDotSolver jnt_to_jac_dot_solver_leftarm(leftarm_chain);
        KDL::ChainIkSolverVel_pinv ik_solver_vel_leftarm(leftarm_chain);
        jnt_to_jac_dot_solver_leftarm.JntToJacDot(jnt_array_vel_leftarm, jd_qd_leftarm);
        xdd_minus_jd_qd_leftarm = xdd - jd_qd_leftarm;
        ik_solver_vel_leftarm.CartToJnt(q_leftarm, xdd_minus_jd_qd_leftarm, qdd_leftarm);

        int r_left = rne_id_solver_leftarm->CartToJnt(
            q_leftarm, qd_leftarm, qdd_leftarm, f_ext_leftarm, tau_ctrl_leftarm
        );
        if (r_left < 0) {
            RCLCPP_ERROR(get_logger(), "Left arm RNE ID solver failed with error code: %d", r_left);
        }
        for (int i = 0; i < num_jnts_leftarm; i++) {
            saturate(&tau_ctrl_leftarm(i), -KINOVA_TAU_CMD_LIMIT, KINOVA_TAU_CMD_LIMIT);
            eddie_state->kinova_leftarm_state.eff_cmd[i] = tau_ctrl_leftarm(i);
        }
    }
}


void EddieRosInterface::execute(events *eventData, EddieState *eddie_state) {
    // RCLCPP_INFO(get_logger(), "In execute state");

    // // Update the EtherCAT state
    // robif2b_ethercat_update(&ecat);
    // if (eddie_state->ecat.error_code < 0) {
    //     RCLCPP_ERROR(get_logger(), "EtherCAT update failed.");
    //     return;
    // }
    // robif2b_kelo_drive_encoder_update(&drive_enc);
    // robif2b_kelo_drive_imu_update(&imu);
    // robif2b_eddie_power_board_update(&power_board);

    for (int i = 0; i < num_jnts_rightarm; i++) {
        q_rightarm(i)  = eddie_state->kinova_rightarm_state.pos_msr[i];
        qd_rightarm(i) = eddie_state->kinova_rightarm_state.vel_msr[i];
    }
    for (int i = 0; i < num_jnts_leftarm; i++) {
        q_leftarm(i)  = eddie_state->kinova_leftarm_state.pos_msr[i];
        qd_leftarm(i) = eddie_state->kinova_leftarm_state.vel_msr[i];
    }

    KDL::JntArrayVel q_qd_rightarm(q_rightarm, qd_rightarm);
    KDL::JntArrayVel q_qd_leftarm(q_leftarm, qd_leftarm);

    KDL::ChainFkSolverPos_recursive fpk_pose_rightarm_ee(rightarm_chain);
    fpk_pose_rightarm_ee.JntToCart(q_rightarm, pose_rightarm_ee);
    KDL::ChainFkSolverVel_recursive fvk_twist_rightarm_ee(rightarm_chain);
    KDL::FrameVel _twist_rightarm_ee;
    fvk_twist_rightarm_ee.JntToCart(q_qd_rightarm, _twist_rightarm_ee);
    twist_rightarm_ee = _twist_rightarm_ee.deriv();

    KDL::ChainFkSolverPos_recursive fpk_pose_leftarm_ee(leftarm_chain);
    fpk_pose_leftarm_ee.JntToCart(q_leftarm, pose_leftarm_ee);
    KDL::ChainFkSolverVel_recursive fvk_twist_leftarm_ee(leftarm_chain);
    KDL::FrameVel _twist_leftarm_ee;
    fvk_twist_leftarm_ee.JntToCart(q_qd_leftarm, _twist_leftarm_ee);
    twist_leftarm_ee = _twist_leftarm_ee.deriv();

    // Compute new target pose
    static bool pose_set = false;
    if (!pose_set) {
        // Offset: move up by 20 cm (0.2 m) in z
        target_pose_wrt_ee = KDL::Vector(0.0, 0.0, 0.2);
        target_pose_offset = KDL::Frame(KDL::Rotation::Identity(), target_pose_wrt_ee);
        KDL::Frame new_target_pose_leftarm_ee = pose_leftarm_ee * target_pose_offset;

        RCLCPP_INFO(get_logger(), "Setting new target pose for left arm: Position: [%f, %f, %f] (Current: [%f, %f, %f])",
                new_target_pose_leftarm_ee.p.x(), new_target_pose_leftarm_ee.p.y(), new_target_pose_leftarm_ee.p.z(),
                pose_leftarm_ee.p.x(), pose_leftarm_ee.p.y(), pose_leftarm_ee.p.z());
        target_pose_leftarm_ee = new_target_pose_leftarm_ee;

        pose_set = true;
    }

    // impedance control for right arm - start pose as target pose
    compute_cartesian_ctrl(eventData, eddie_state);

    // robif2b_kelo_drive_actuator_update(&wheel_act);
    if (param_arm_to_control == "rightarm") {
        robif2b_kinova_gen3_update(&kinova_rightarm);
    } else if (param_arm_to_control == "leftarm") {
        robif2b_kinova_gen3_update(&kinova_leftarm);
    }
}

void EddieRosInterface::fsm_behavior(events *eventData, EddieState *eddie_state) {
    if (consume_event(eventData, E_CONFIGURE_ENTERED)) {
        configure(eventData, eddie_state);
    }

    if (consume_event(eventData, E_IDLE_ENTERED)) {
        idle(eventData, eddie_state);
    }

    if (consume_event(eventData, E_COMPILE_ENTERED)) {
        compile(eventData, eddie_state);
    }

    if (consume_event(eventData, E_EXECUTE_ENTERED)) {
        execute(eventData, eddie_state);
    }
}

void EddieRosInterface::run_fsm() {
    auto rate = rclcpp::Rate(1.0 / 0.001); // 1 kHz

    while (rclcpp::ok() && keep_running) {
        clock_gettime(CLOCK_MONOTONIC, &eddie_state.time.cycle_start);

        if (fsm.currentStateIndex == S_EXIT) {
            break;
        }

        produce_event(&eventData, E_STEP);

        fsm_behavior(&eventData, &eddie_state);
        fsm_step_nbx(&fsm);
        reconfig_event_buffers(&eventData);

        // rclcpp::spin_some(this->shared_from_this());

        clock_gettime(CLOCK_MONOTONIC, &eddie_state.time.cycle_end);
        eddie_state.time.cycle_time_msr = timespec_to_usec(&eddie_state.time.cycle_end) -
                                          timespec_to_usec(&eddie_state.time.cycle_start);
        rate.sleep();
    }

    RCLCPP_INFO(get_logger(), "Eddie ROS interface node shutting down.");
    // robif2b_kelo_drive_actuator_stop(&wheel_act);
    // robif2b_ethercat_stop(&ecat);
    // robif2b_ethercat_shutdown(&ecat);
    // if (eddie_state.ecat.error_code < 0) {
    //     RCLCPP_ERROR(get_logger(), "EtherCAT stop failed.");
    //     return;
    // }

    if (param_arm_to_control == "rightarm") {
        RCLCPP_INFO(get_logger(), "Shutting down right arm");
        robif2b_kinova_gen3_stop(&kinova_rightarm);
        robif2b_kinova_gen3_shutdown(&kinova_rightarm);
    } else if (param_arm_to_control == "leftarm") {
        RCLCPP_INFO(get_logger(), "Shutting down left arm");
        robif2b_kinova_gen3_stop(&kinova_leftarm);
        robif2b_kinova_gen3_shutdown(&kinova_leftarm);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EddieRosInterface>(rclcpp::NodeOptions());

    node->run_fsm();

    rclcpp::shutdown();
    return 0;
}