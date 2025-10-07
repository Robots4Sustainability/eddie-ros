#include "eddie_ros/interface.hpp"
#include <signal.h>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "eddie_ros/action/arm_control.hpp"
#include "eddie_ros/action/gripper_control.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <kdl_parser/kdl_parser.hpp>

volatile sig_atomic_t keep_running = 1;

// helper functions
// replace these later

template<typename PoseType>
KDL::Frame poseToKDL(const PoseType& pose) {
    KDL::Vector position(pose.position.x, pose.position.y, pose.position.z);
    KDL::Rotation rotation = KDL::Rotation::Quaternion(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    );
    return KDL::Frame(rotation, position);
}

template<typename PoseType>
PoseType kdlToPose(const KDL::Frame& frame) {
    PoseType pose;
    pose.position.x = frame.p.x();
    pose.position.y = frame.p.y();
    pose.position.z = frame.p.z();
    
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    pose.orientation.x = x;
    pose.orientation.y = y;
    pose.orientation.z = z;
    pose.orientation.w = w;
    
    return pose;
}

class SimInterfaceNode : public rclcpp::Node
{
public:
    using ArmControl = eddie_ros::action::ArmControl;
    using GripperControl = eddie_ros::action::GripperControl;
    using GoalHandleArmControl = rclcpp_action::ServerGoalHandle<ArmControl>;
    using GoalHandleGripperControl = rclcpp_action::ServerGoalHandle<GripperControl>;

    SimInterfaceNode() : Node("sim_interface_node")
    {
        // Parameter Setup
        //declare the robot_description parameter
        this->declare_parameter<std::string>("robot_description", "");
        param_arm_select_ = this->declare_parameter("arm_select", "both");
        RCLCPP_INFO(this->get_logger(), "Simulation started for arm_select: '%s'", param_arm_select_.c_str());

        // KDL Setup from robot_description Parameter
        RCLCPP_INFO(this->get_logger(), "Loading robot model from 'robot_description' parameter...");
        
        // Get the URDF string from the parameter server
        std::string urdf_string = this->get_parameter("robot_description").as_string();
        
        if (urdf_string.empty()) {
            RCLCPP_FATAL(this->get_logger(), "'robot_description' parameter is empty. Please provide a URDF.");
            rclcpp::shutdown();
            return;
        }

        // Parse the URDF string directly into a KDL tree
        if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to construct KDL tree from URDF string.");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded KDL tree from parameter.");

        if (should_control_right_arm()) {
            /*if (!kdl_tree_.getChain("base_link", "kinova_right_grasp_link", right_arm_chain_)) {
                RCLCPP_FATAL(this->get_logger(), "Failed to get KDL chain for right arm.");
                rclcpp::shutdown();
                return;
            }*/
           if (!kdl_tree_.getChain("eddie_base_link", "eddie_right_arm_robotiq_85_grasp_link", right_arm_chain_)) {
                RCLCPP_FATAL(this->get_logger(), "Failed to get KDL chain for right arm.");
                rclcpp::shutdown();
                return;
            }
            right_arm_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(right_arm_chain_);
            right_arm_ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(right_arm_chain_);
            right_arm_joint_positions_.resize(right_arm_chain_.getNrOfJoints());

            // right_arm_joint_positions_.data.setZero(); // Start at home position (TODO: define home position)
            
            right_arm_joint_positions_.data(0) = 0.70;
            right_arm_joint_positions_.data(1) = -2.05;
            right_arm_joint_positions_.data(2) = 0.90;
            right_arm_joint_positions_.data(3) = 2.44;
            right_arm_joint_positions_.data(4) = 1.57;
            right_arm_joint_positions_.data(5) = 0.00;
            right_arm_joint_positions_.data(6) = 0.00;

            right_gripper_joint_name_ = "eddie_right_arm_robotiq_85_left_knuckle_joint";
            right_gripper_position_ = 0.0; // Start fully open
        }
        if (should_control_left_arm()) {
            if (!kdl_tree_.getChain("eddie_base_link", "eddie_left_arm_robotiq_85_grasp_link", left_arm_chain_)) {
                RCLCPP_FATAL(this->get_logger(), "Failed to get KDL chain for left arm.");
                rclcpp::shutdown();
                return;
            }
            left_arm_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(left_arm_chain_);
            left_arm_ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(left_arm_chain_);
            left_arm_joint_positions_.resize(left_arm_chain_.getNrOfJoints());
            
            //left_arm_joint_positions_.data.setZero(); // Start at home position

            left_arm_joint_positions_.data(0) = -0.70;
            left_arm_joint_positions_.data(1) = -2.05;
            left_arm_joint_positions_.data(2) = 2.14;
            left_arm_joint_positions_.data(3) = -2.44;
            left_arm_joint_positions_.data(4) = -1.57;
            left_arm_joint_positions_.data(5) = 0.00;
            left_arm_joint_positions_.data(6) = 0.00;

            left_gripper_joint_name_ = "eddie_left_arm_robotiq_85_left_knuckle_joint";
            left_gripper_position_ = 0.0; // Start fully open
        }

        // ROS
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50 Hz
            std::bind(&SimInterfaceNode::publish_joint_states, this));

        if (should_control_right_arm()) {
            right_arm_action_server_ = rclcpp_action::create_server<ArmControl>(this, "right_arm/arm_control",
                std::bind(&SimInterfaceNode::handle_arm_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&SimInterfaceNode::handle_arm_cancel, this, std::placeholders::_1),
                std::bind(&SimInterfaceNode::handle_right_arm_accepted, this, std::placeholders::_1));
            right_gripper_action_server_ = rclcpp_action::create_server<GripperControl>(this, "right_arm/gripper_control",
                std::bind(&SimInterfaceNode::handle_gripper_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&SimInterfaceNode::handle_gripper_cancel, this, std::placeholders::_1),
                std::bind(&SimInterfaceNode::handle_right_gripper_accepted, this, std::placeholders::_1));
        }
        if (should_control_left_arm()) {
            left_arm_action_server_ = rclcpp_action::create_server<ArmControl>(this, "left_arm/arm_control",
                std::bind(&SimInterfaceNode::handle_arm_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&SimInterfaceNode::handle_arm_cancel, this, std::placeholders::_1),
                std::bind(&SimInterfaceNode::handle_left_arm_accepted, this, std::placeholders::_1));

            left_gripper_action_server_ = rclcpp_action::create_server<GripperControl>(this, "left_arm/gripper_control",
                std::bind(&SimInterfaceNode::handle_gripper_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&SimInterfaceNode::handle_gripper_cancel, this, std::placeholders::_1),
                std::bind(&SimInterfaceNode::handle_left_gripper_accepted, this, std::placeholders::_1));
        }
        
        RCLCPP_INFO(this->get_logger(), "Simulation Interface Node has started. Ready for goals.");
    }

private:
    // Parameters
    std::string param_arm_select_;

    // KDL Members
    KDL::Tree kdl_tree_;
    KDL::Chain right_arm_chain_, left_arm_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> right_arm_fk_solver_, left_arm_fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> right_arm_ik_vel_solver_, left_arm_ik_vel_solver_;

    // State Variables
    KDL::JntArray right_arm_joint_positions_, left_arm_joint_positions_;
    bool is_right_arm_busy_ = false, is_left_arm_busy_ = false;

    std::string right_gripper_joint_name_, left_gripper_joint_name_;
    double right_gripper_position_, left_gripper_position_;
    
    // ROS Components
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp_action::Server<ArmControl>::SharedPtr right_arm_action_server_, left_arm_action_server_;
    rclcpp_action::Server<GripperControl>::SharedPtr right_gripper_action_server_, left_gripper_action_server_;

    // Helper Functions
    bool should_control_right_arm() const { return param_arm_select_ == "right" || param_arm_select_ == "both"; }
    bool should_control_left_arm() const { return param_arm_select_ == "left" || param_arm_select_ == "both"; }

    // Publisher Callback
    void publish_joint_states() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();
        
        if (should_control_right_arm()) {
            // const std::vector<std::string> right_names = { "kinova_right_joint_1", "kinova_right_joint_2", "kinova_right_joint_3", "kinova_right_joint_4", "kinova_right_joint_5", "kinova_right_joint_6", "kinova_right_joint_7"};
            const std::vector<std::string> right_names = {
            "eddie_right_arm_joint_1", "eddie_right_arm_joint_2", "eddie_right_arm_joint_3",
            "eddie_right_arm_joint_4", "eddie_right_arm_joint_5", "eddie_right_arm_joint_6", "eddie_right_arm_joint_7"
            };
            for (unsigned int i = 0; i < right_arm_joint_positions_.rows(); ++i) {
                msg.name.push_back(right_names[i]);
                msg.position.push_back(right_arm_joint_positions_(i));
            }
            
            // Convert the 0-100 command to the 0.0-0.8 radian range.
            // 0.0 = open, 0.8 = closed.
            // 0 = open, 100 = closed.
            msg.name.push_back(right_gripper_joint_name_);
            double joint_value_radians = right_gripper_position_ * (0.8 / 100.0);
            msg.position.push_back(joint_value_radians);
        }
        if (should_control_left_arm()) {
            const std::vector<std::string> left_names = {
            "eddie_left_arm_joint_1", "eddie_left_arm_joint_2", "eddie_left_arm_joint_3",
            "eddie_left_arm_joint_4", "eddie_left_arm_joint_5", "eddie_left_arm_joint_6", "eddie_left_arm_joint_7"
            };
            for (unsigned int i = 0; i < left_arm_joint_positions_.rows(); ++i) {
                msg.name.push_back(left_names[i]);
                msg.position.push_back(left_arm_joint_positions_(i));
            }
            msg.name.push_back(left_gripper_joint_name_);
            double joint_value_radians = left_gripper_position_ * (0.8 / 100.0);
            msg.position.push_back(joint_value_radians);
        }
        if (!msg.name.empty()) {
            joint_state_publisher_->publish(msg);
        }
    }

    // Action Server Callbacks
    rclcpp_action::GoalResponse handle_arm_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmControl::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_arm_cancel(const std::shared_ptr<GoalHandleArmControl>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_right_arm_accepted(const std::shared_ptr<GoalHandleArmControl> goal_handle) {
        if (is_right_arm_busy_) {
            RCLCPP_WARN(this->get_logger(), "Right arm is busy, rejecting goal.");
            goal_handle->abort(std::make_shared<ArmControl::Result>());
            return;
        }
        is_right_arm_busy_ = true;
        std::thread{std::bind(&SimInterfaceNode::simulate_kinematic_motion, this, std::placeholders::_1, true), goal_handle}.detach();
    }

    void handle_left_arm_accepted(const std::shared_ptr<GoalHandleArmControl> goal_handle) {
        if (is_left_arm_busy_) {
            RCLCPP_WARN(this->get_logger(), "Left arm is busy, rejecting goal.");
            goal_handle->abort(std::make_shared<ArmControl::Result>());
            return;
        }
        is_left_arm_busy_ = true;
        std::thread{std::bind(&SimInterfaceNode::simulate_kinematic_motion, this, std::placeholders::_1, false), goal_handle}.detach();
    }
    
    // KINEMATIC SIMULATION FUNCTION
    void simulate_kinematic_motion(const std::shared_ptr<GoalHandleArmControl> goal_handle, bool is_right_arm)
    {
        // Select the correct chain, solvers, and state based on the arm
        KDL::Chain& chain = is_right_arm ? right_arm_chain_ : left_arm_chain_;
        auto& fk_solver = is_right_arm ? right_arm_fk_solver_ : left_arm_fk_solver_;
        auto& ik_vel_solver = is_right_arm ? right_arm_ik_vel_solver_ : left_arm_ik_vel_solver_;
        KDL::JntArray& joint_positions = is_right_arm ? right_arm_joint_positions_ : left_arm_joint_positions_;
        
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ArmControl::Result>();
        
        KDL::Frame current_frame;
        fk_solver->JntToCart(joint_positions, current_frame);

        // assumes relative goals
        KDL::Frame relative_goal_frame = poseToKDL(goal->target_pose);
        KDL::Frame target_frame = current_frame * relative_goal_frame;

        rclcpp::Rate loop_rate(50); // Control loop at 50Hz
        double duration = 4.0; 
        int num_steps = duration * 50;
        
        for (int i = 0; i < num_steps; ++i) {
            if (goal_handle->is_canceling()) {
                is_right_arm ? is_right_arm_busy_ = false : is_left_arm_busy_ = false;
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            fk_solver->JntToCart(joint_positions, current_frame);
            KDL::Twist cartesian_error = KDL::diff(current_frame, target_frame);
            
            if (cartesian_error.vel.Norm() < 0.01 && cartesian_error.rot.Norm() < 0.05) {
                break; // Success
            }

            KDL::JntArray joint_velocities(chain.getNrOfJoints());
            ik_vel_solver->CartToJnt(joint_positions, cartesian_error, joint_velocities);

            double dt = 1.0 / 50.0;
            for (unsigned int j = 0; j < joint_positions.rows(); ++j) {
                joint_positions(j) += joint_velocities(j) * dt;
            }
            
            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
            is_right_arm ? is_right_arm_busy_ = false : is_left_arm_busy_ = false;
            result->success = true;
            goal_handle->succeed(result);
        }
    }

    // Gripper Action Server Callbacks
    rclcpp_action::GoalResponse handle_gripper_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GripperControl::Goal> )
    {
        RCLCPP_INFO(this->get_logger(), "Received gripper goal request. Accepting.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_gripper_cancel(
        const std::shared_ptr<GoalHandleGripperControl> )
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel gripper goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_right_gripper_accepted(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
    {
        // Spawn a thread to simulate the gripper motion.
        std::thread{std::bind(&SimInterfaceNode::simulate_gripper_motion, this, std::placeholders::_1, true), goal_handle}.detach();
    }

   void handle_left_gripper_accepted(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
    {
        std::thread{std::bind(&SimInterfaceNode::simulate_gripper_motion, this, std::placeholders::_1, false), goal_handle}.detach();
    }
    
    void simulate_gripper_motion(const std::shared_ptr<GoalHandleGripperControl> goal_handle, bool is_right_gripper)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<GripperControl::Result>();

        double& gripper_position = is_right_gripper ? right_gripper_position_ : left_gripper_position_;

        double start_position = gripper_position;
        double target_position = std::clamp(goal->target_position, 0.0, 100.0);

        RCLCPP_INFO(this->get_logger(), "Simulating gripper move from %.1f to %.1f", start_position, target_position);

        // Animate the gripper over 1 second
        rclcpp::Rate loop_rate(50);
        int num_steps = 50; // 50 steps * (1/50s) = 1 second
        double step_increment = (target_position - start_position) / num_steps;

        for (int i = 0; i < num_steps; ++i) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            gripper_position += step_increment;
            loop_rate.sleep();
        }
        
        gripper_position = target_position; // Ensure it ends at the exact target

        if (rclcpp::ok()) {
            result->success = true;
            result->final_position = gripper_position;
            goal_handle->succeed(result);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}