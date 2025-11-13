#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  // --- 1️⃣ Initialize ROS and create a Node ---
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "moveit_controll",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::Logger logger = rclcpp::get_logger("moveit_controll");

  // --- 2️⃣ Create MoveGroupInterface for your planning group ---
  static const std::string PLANNING_GROUP = "david_arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // --- 3️⃣ Setup MoveIt Visual Tools ---
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", rvt::RVIZ_MARKER_TOPIC);
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // --- 4️⃣ Choose target pose ---
  geometry_msgs::msg::Pose target_pose = move_group.getCurrentPose().pose;
  target_pose.position.z += 0.10;  // Move up 10 cm
  move_group.setPoseTarget(target_pose);

  // Visualize target pose
  visual_tools.publishAxisLabeled(target_pose, "target_pose");
  visual_tools.trigger();

  // --- 5️⃣ Plan motion ---
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // --- 6️⃣ Execute ---
  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful, executing plan...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // --- 7️⃣ Shutdown cleanly ---
  rclcpp::shutdown();
  return 0;
}
