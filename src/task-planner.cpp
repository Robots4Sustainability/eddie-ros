#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "eddie_ros/action/arm_control.hpp"
#include "eddie_ros/action/gripper_control.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


// Client Node that performs a sequence of actions
// in this example, the aim is to grasp an object:
// 1. move to an approach pose
// 2. move forward to the final grasping pose (make it slower? not sure how to set speed here)
// 3. close the gripper
class TaskPlannerNode : public rclcpp::Node
{
public:
    using ArmControl = eddie_ros::action::ArmControl;
    using GripperControl = eddie_ros::action::GripperControl;
    using GoalHandleArmControl = rclcpp_action::ClientGoalHandle<ArmControl>;
    using GoalHandleGripperControl = rclcpp_action::ClientGoalHandle<GripperControl>;

    explicit TaskPlannerNode(const rclcpp::NodeOptions & options) : Node("task_planner_node", options)
    {
        // Action clients (only for the RIGHT arm for now)
        this->arm_client_ptr_ = rclcpp_action::create_client<ArmControl>(this, "right_arm/arm_control");
        this->gripper_client_ptr_ = rclcpp_action::create_client<GripperControl>(this, "right_arm/gripper_control");

        // Subscriber to the perception topic
        this->perception_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/perception/target_pose", 10,
            std::bind(&TaskPlannerNode::perception_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Task Planner initialized. Waiting for perception data...");
    }

private:
    rclcpp_action::Client<ArmControl>::SharedPtr arm_client_ptr_;
    rclcpp_action::Client<GripperControl>::SharedPtr gripper_client_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr perception_subscriber_;

    // State variables
    bool task_in_progress_ = false;
    geometry_msgs::msg::PoseStamped last_perceived_pose_;

    // triggered whenever the perception node publishes a new pose.
    void perception_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // If already running a task, ignore new perception data until it is done.
        if (task_in_progress_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received new target pose from perception. Starting disassembly sequence.");
        task_in_progress_ = true;
        last_perceived_pose_ = *msg;

        // Start the sequence of actions
        move_to_approach_pose();
    }

    // STEP 1: move to an approach pose (in front of the target? not sure which position (x, y, z) exactly)
    void move_to_approach_pose()
    {
        // TODO: transform the perceived pose from the camera's frame to the robot's base frame using tf2.
        // for now assume it's already in the right frame
        
        auto goal_msg = ArmControl::Goal();
        goal_msg.target_pose = last_perceived_pose_.pose;

        // Create an approach pose that is 10cm in front of the actual target
        goal_msg.target_pose.position.z -= 0.10;

        RCLCPP_INFO(this->get_logger(), "STEP 1: Moving to approach pose.");
        
        auto send_goal_options = rclcpp_action::Client<ArmControl>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleArmControl::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                this->move_to_grasp_pose(); // If successful, move to the next step
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to approach pose. Aborting.");
                this->task_in_progress_ = false;
            }
        };
        this->arm_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // STEP 2: Move forward to the final grasping pose
    void move_to_grasp_pose()
    {
        auto goal_msg = ArmControl::Goal();
        // move forward by 10cm from the approach pose
        goal_msg.target_pose.position.z = +0.10;

        RCLCPP_INFO(this->get_logger(), "STEP 2: Moving to grasp pose.");

        auto send_goal_options = rclcpp_action::Client<ArmControl>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleArmControl::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                this->close_gripper(); // If successful, close the gripper
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to grasp pose. Aborting.");
                this->task_in_progress_ = false;
            }
        };
        this->arm_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    // STEP 3: Close the gripper
    /* TODO: idk how the gripper works exactly: 
        - how does it detect if the object is grasped or not? 
        - is the task only successful if the gripper is fully closed? 
    */
    void close_gripper()
    {
        auto goal_msg = GripperControl::Goal();
        goal_msg.target_position = 100.0; // Close fully
        goal_msg.velocity = 20.0; // Some reasonable speed
        goal_msg.force = 10.0; // Some reasonable force

        RCLCPP_INFO(this->get_logger(), "STEP 3: Closing gripper.");
        
        auto send_goal_options = rclcpp_action::Client<GripperControl>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleGripperControl::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Grasp sequence complete!");
                this->task_in_progress_ = false; // Allow a new task to start
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to close gripper. Aborting.");
                this->task_in_progress_ = false;
            }
        };
        this->gripper_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto task_planner = std::make_shared<TaskPlannerNode>(rclcpp::NodeOptions());
    rclcpp::spin(task_planner);
    rclcpp::shutdown();
    return 0;
}