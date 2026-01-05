#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class SimpleMotionNode : public rclcpp::Node
{
public:
    SimpleMotionNode()
        : Node("simple_motion_node")
    {
        RCLCPP_INFO(this->get_logger(), "SimpleMotionNode created.");

        // Create a timer to initialize MoveIt AFTER the node is fully constructed
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SimpleMotionNode::initialize_moveit, this)
        );
    }

private:
    void initialize_moveit()
    {
        // Run this only once
        timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interfaces...");

        // SAFE: now shared_from_this() is valid
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(),
            "david_arm"
        );

        planning_scene_interface_ =
            std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        move_group_->setPlanningTime(5.0);
        move_group_->setPoseReferenceFrame("base_link");

        // // ---- Target pose ----
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.0;
        target_pose.position.y = 0.096;
        target_pose.position.z = 0.36;

        move_group_->setPoseTarget(target_pose);

        // // ---- Plan ----
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_->plan(plan);

        if (result.val == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");
            move_group_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }

    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMotionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
