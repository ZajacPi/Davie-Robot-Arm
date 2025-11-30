#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Command2Motion : public rclcpp::Node
{
public:
    Command2Motion() : Node("command2motion_node")
    {
        RCLCPP_INFO(this->get_logger(), "command2motion Node created.");

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "movement_cmd",
            10,
            std::bind(&Command2Motion::twist_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Command2Motion::initialize_moveit, this)
        );
    }

private:
    void initialize_moveit()
    {
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interfaces...");

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),"david_arm");

        planning_scene_interface_ =std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        move_group_->setPlanningTime(5.0);
        move_group_->setPoseReferenceFrame("base_link");

        RCLCPP_INFO(this->get_logger(), "MoveIt Initialized.");
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
    //     if (!move_group_) {
    //         RCLCPP_WARN(this->get_logger(), "MoveIt not ready yet");
    //         return;
    //     }

    //     RCLCPP_INFO(this->get_logger(), "Received voice command.");
    // // Wait until current state is valid
    //     auto current_state = move_group_->getCurrentState();
    //     if (!current_state) {
    //         RCLCPP_WARN(this->get_logger(), "Current robot state not received yet, skipping command.");
    //         return;
    //     }
        auto current_pose = move_group_->getCurrentPose().pose;
        auto target_pose = current_pose;

        double dx = msg->linear.x * 0.001;
        double dy = msg->linear.y * 0.001;
        double dz = msg->linear.z * 0.001;

        target_pose.position.x += dx;
        target_pose.position.y += dy;
        target_pose.position.z += dz;

        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(current_pose.orientation, q_orig);

        q_rot.setRPY(msg->angular.x, msg->angular.y, msg->angular.z);
        q_new = q_orig * q_rot;
        target_pose.orientation = tf2::toMsg(q_new);

        // Cartesian path
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction < 0.1)
        {
            RCLCPP_ERROR(this->get_logger(), "Cartesian path failed: fraction = %.2f", fraction);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Executing relative motion (success fraction = %.2f)", fraction);

        move_group_->execute(trajectory);
    }
    // void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    // {
    //     if (!move_group_) {
    //         RCLCPP_WARN(this->get_logger(), "MoveIt not ready yet");
    //         return;
    //     }

    //     // Wait until current state is valid
    //     auto current_state = move_group_->getCurrentState();
    //     if (!current_state) {
    //         RCLCPP_WARN(this->get_logger(), "Current robot state not received yet, skipping command.");
    //         return;
    //     }

    //     // Get current pose
    //     auto current_pose_stamped = move_group_->getCurrentPose();
    //     auto target_pose = current_pose_stamped.pose;

    //     // Apply relative linear displacement (meters)
    //     target_pose.position.x += msg->linear.x * 0.001;
    //     target_pose.position.y += msg->linear.y * 0.001;
    //     target_pose.position.z += msg->linear.z * 0.001;

    //     // Apply relative rotation (radians)
    //     tf2::Quaternion q_orig, q_rot, q_new;
    //     tf2::fromMsg(current_pose_stamped.pose.orientation, q_orig);
    //     q_rot.setRPY(msg->angular.x, msg->angular.y, msg->angular.z);
    //     q_new = q_orig * q_rot;
    //     target_pose.orientation = tf2::toMsg(q_new);

    //     // Set the new pose target
    //     move_group_->setPoseTarget(target_pose);

    //     // Plan and execute motion
    //     moveit::planning_interface::MoveGroupInterface::Plan plan;
    //     auto success = move_group_->plan(plan);

    //     if (success == moveit::core::MoveItErrorCode::SUCCESS) {
    //         RCLCPP_INFO(this->get_logger(), "Executing planned motion...");
    //         move_group_->execute(plan);
    //     } else {
    //         RCLCPP_ERROR(this->get_logger(), "Planning failed, skipping command.");
    //     }

    //     // Clear target to avoid accumulation
    //     move_group_->clearPoseTargets();
    // }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Command2Motion>());
    rclcpp::shutdown();
    return 0;
};
