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
    if (!tree.getChain("eddie_base_link", "eddie_left_arm_end_effector_link", leftarm_chain)) {
        RCLCPP_ERROR(get_logger(), "Failed to get left arm chain");
        exit(11);
    } else {
        RCLCPP_INFO(get_logger(), "Left arm chain constructed successfully");
    }
    if (!tree.getChain("eddie_base_link", "eddie_right_arm_end_effector_link", rightarm_chain)) {
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

    num_jnts_leftarm = leftarm_chain.getNrOfJoints();
    num_segs_leftarm = leftarm_chain.getNrOfSegments();
    root_acc_leftarm = KDL::Twist(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector::Zero());
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

    // std::cout << "Left arm chain: " << leftarm_chain.getNrOfJoints() << " joints, "
    //           << leftarm_chain.getNrOfSegments() << " segments" << std::endl;
    // std::cout << "Left arm segments: " << std::endl;
    // for (int i = 0; i < num_segs_leftarm; i++) {
    //     std::cout << leftarm_chain.getSegment(i).getName() << std::endl;
    // }
    // std::cout << "Right arm chain: " << rightarm_chain.getNrOfJoints() << " joints, "
    //           << rightarm_chain.getNrOfSegments() << " segments" << std::endl;
    // std::cout << "Right arm segments: " << std::endl;
    // for (int i = 0; i < num_segs_rightarm; i++) {
    //     std::cout << rightarm_chain.getSegment(i).getName() << std::endl;
    // }

    pid_rightarm_ee_pos_x.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_pos_y.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_pos_z.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_rot_x.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_rot_y.set_gains(50.0, 0., 0.0, 0.9);
    pid_rightarm_ee_rot_z.set_gains(50.0, 0., 0.0, 0.9);

    RCLCPP_INFO(get_logger(), "Eddie ROS interface node initialized.");
}

EddieRosInterface::~EddieRosInterface() {}

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

    kinova_rightarm.conf.ip_address         = "192.168.1.12";
    kinova_rightarm.conf.port               = 10000;
    kinova_rightarm.conf.port_real_time     = 10001;
    kinova_rightarm.conf.user               = "admin";
    kinova_rightarm.conf.password           = "admin";
    kinova_rightarm.conf.session_timeout    = 60000;
    kinova_rightarm.conf.connection_timeout = 2000;
    double cycle_time                       = 0.001;
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
    kinova_rightarm.imu_ang_vel_msr = &eddie_state->kinova_rightarm_state.imu_ang_vel_msr[0];
    kinova_rightarm.imu_lin_acc_msr = &eddie_state->kinova_rightarm_state.imu_lin_acc_msr[0];

    RCLCPP_INFO(get_logger(), "Eddie ROS interface configured.");

    RCLCPP_DEBUG(get_logger(), "In configure state");
    produce_event(eventData, E_CONFIGURE_EXIT);
}

void EddieRosInterface::idle(events *eventData, const EddieState *eddie_state) {
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

    RCLCPP_DEBUG(get_logger(), "Exiting idle state");
    produce_event(eventData, E_IDLE_EXIT_EXECUTE);
}

void EddieRosInterface::compile(events *eventData, const EddieState *eddie_state) {

    RCLCPP_DEBUG(get_logger(), "Exiting compile state");
    produce_event(eventData, E_COMPILE_EXIT);
}

void EddieRosInterface::compute_gravity_comp(events *eventData, EddieState *eddie_state) {
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
}

void EddieRosInterface::compute_cartesian_ctrl(events *eventData, EddieState *eddie_state) {

    KDL::Twist delta_pose_rightarm_ee = KDL::diff(target_pose_rightarm_ee, pose_rightarm_ee);

    long cycle_time_msr = eddie_state->time.cycle_time_msr;

    // convert to seconds
    double cycle_time = static_cast<double>(cycle_time_msr) / 1e6;
    if (cycle_time <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Invalid cycle time: %ld", cycle_time_msr);
        return;
    }

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
}

void EddieRosInterface::execute(events *eventData, EddieState *eddie_state) {
    // RCLCPP_INFO(get_logger(), "In execute state");

    // simulate data read
    double rightarm_jpos[NUM_JOINTS] = {
        4.78218, -0.977604, 4.93074, -1.66029, 4.43362, -0.323419, 5.76011
    };
    for (int i = 0; i < num_jnts_rightarm; i++) {
        eddie_state->kinova_rightarm_state.pos_msr[i] = rightarm_jpos[i];
    }

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

    // compute gravity compensation torques using the RNE ID solver
    compute_gravity_comp(eventData, eddie_state);

    // impedance control for right arm - start pose as target pose
    // compute_cartesian_ctrl(eventData, eddie_state);

    printf("Right arm tau cmd: ");
    for (int i = 0; i < num_jnts_rightarm; i++) {
        printf("%f ", eddie_state->kinova_rightarm_state.eff_cmd[i]);
    }
    printf("\n");

    exit(45);
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
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EddieRosInterface>(rclcpp::NodeOptions());

    node->run_fsm();

    rclcpp::shutdown();
    return 0;
}