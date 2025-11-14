#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_move");

  // Create MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface move_group(node, "david_arm");

  // Print debugging info
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Set a simple target (move up)
  auto pose = move_group.getCurrentPose().pose;
  pose.position.z += 0.10;  // 10cm up

  move_group.setPoseTarget(pose);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group.plan(plan);

  if (result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Plan OK, executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning FAILED.");
  }

  // Shutdown
  rclcpp::shutdown();
  return 0;
}
