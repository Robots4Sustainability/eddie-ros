#include "eddie_ros/interface.hpp"
#include <signal.h>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "eddie_ros/action/arm_control.hpp"
#include "eddie_ros/action/gripper_control.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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
        param_arm_select_ = this->declare_parameter("arm_select", "both");
        RCLCPP_INFO(this->get_logger(), "Simulation started for arm_select: '%s'", param_arm_select_.c_str());

        // KDL Setup
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("eddie_ros");
        std::string urdf_path = package_share_directory + "/urdf/eddie.urdf";
        if (!kdl_parser::treeFromFile(urdf_path, kdl_tree_)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to construct KDL tree from URDF.");
            rclcpp::shutdown();
            return;
        }
        if (should_control_right_arm()) {
            /*if (!kdl_tree_.getChain("base_link", "kinova_right_grasp_link", right_arm_chain_)) {
                RCLCPP_FATAL(this->get_logger(), "Failed to get KDL chain for right arm.");
                rclcpp::shutdown();
                return;
            }*/
           if (!kdl_tree_.getChain("eddie_base_link", "eddie_right_arm_bracelet_link", right_arm_chain_)) {
                RCLCPP_FATAL(this->get_logger(), "Failed to get KDL chain for right arm.");
                rclcpp::shutdown();
                return;
            }
            right_arm_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(right_arm_chain_);
            right_arm_ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(right_arm_chain_);
            right_arm_joint_positions_.resize(right_arm_chain_.getNrOfJoints());
            right_arm_joint_positions_.data.setZero(); // Start at home position
        }

        // Add left arm

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
        }

        // Add for left arm server
        
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
    
    // ROS Components
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp_action::Server<ArmControl>::SharedPtr right_arm_action_server_, left_arm_action_server_;
    // gripper servers

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
        }

        // Add left arm

        if (!msg.name.empty()) {
            joint_state_publisher_->publish(msg);
        }
    }

    // Action Server Callbacks
    rclcpp_action::GoalResponse handle_arm_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmControl::Goal>) {
        // generic goal handler, check the busy flag in the accepted handler
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
    
    // Add handle_left_arm_accepted

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
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}