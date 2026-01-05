#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

const char* RED     = "\033[1;31m";
const char* GREEN   = "\033[1;32m";
const char* RESET   = "\033[0m";


class Command2Motion : public rclcpp::Node
{
public:
    Command2Motion()
        : Node("command2motion_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
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

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(),
            "david_arm"
        );

        planning_scene_interface_ =
            std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        move_group_->setPlanningTime(5.0);
        move_group_->setPoseReferenceFrame("base_link");

        RCLCPP_INFO(this->get_logger(), "MoveIt Initialized.");
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!move_group_) {
            RCLCPP_WARN(this->get_logger(), "MoveIt not ready yet");
            return;
        }

        geometry_msgs::msg::Pose current_pose;
        geometry_msgs::msg::Pose target_pose;

        // Get current TCP pose via TF
        try {
            auto tf_stamped = tf_buffer_.lookupTransform(
                "base_link", // reference frame
                "tool_1",    // target frame
                tf2::TimePointZero
            );

            current_pose.position.x = tf_stamped.transform.translation.x;
            current_pose.position.y = tf_stamped.transform.translation.y;
            current_pose.position.z = tf_stamped.transform.translation.z;

            current_pose.orientation = tf2::toMsg(tf2::Quaternion(
                tf_stamped.transform.rotation.x,
                tf_stamped.transform.rotation.y,
                tf_stamped.transform.rotation.z,
                tf_stamped.transform.rotation.w
            ));

            // RCLCPP_INFO(this->get_logger(),
            //     "TCP Position: x=%.3f y=%.3f z=%.3f",
            //     current_pose.position.x,
            //     current_pose.position.y,
            //     current_pose.position.z
            // );

            // RCLCPP_INFO(this->get_logger(),
            //     "TCP Orientation: x=%.3f y=%.3f z=%.3f w=%.3f",
            //     current_pose.orientation.x,
            //     current_pose.orientation.y,
            //     current_pose.orientation.z,
            //     current_pose.orientation.w
            // );

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "%sCould not get TCP transform: %s",RED, RESET, ex.what());
            return;
        }

        // Compute target pose relative to current TCP
        target_pose = current_pose;
        target_pose.position.x += msg->linear.x * 0.001;
        target_pose.position.y += msg->linear.y * 0.001;
        target_pose.position.z += msg->linear.z * 0.001;

        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(current_pose.orientation, q_orig);
        q_rot.setRPY(msg->angular.x, msg->angular.y, msg->angular.z);
        q_new = q_orig * q_rot;
        target_pose.orientation = tf2::toMsg(q_new);

        // Set pose target
        move_group_->setPoseTarget(target_pose);

        // Plan motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_->plan(plan);

        if (result.val == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Executing relative movement...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "%sPlanning failed.%s", RED, RESET);
        }

        move_group_->clearPoseTargets();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Command2Motion>());
    rclcpp::shutdown();
    return 0;
}
