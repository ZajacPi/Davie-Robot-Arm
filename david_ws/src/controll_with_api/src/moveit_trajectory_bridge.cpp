#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class MoveItTrajectoryBridge : public rclcpp::Node
{
public:
    MoveItTrajectoryBridge() 
        : Node("moveit_trajectory_bridge")
    {
        using std::placeholders::_1;

        // Publisher to micro-ROS on ESP32
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/joints_cmd", 10);

        // Subscription to trajectory
        sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/david_arm_controller/joint_trajectory", 10,
            std::bind(&MoveItTrajectoryBridge::trajectoryCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), 
            "MoveIt trajectory bridge initialized (C++).");
    }

private:

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;

    void trajectoryCallback(
        const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory.");
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Received trajectory with %zu points.", msg->points.size());

        auto start_time = this->now();

        for (size_t i = 0; i < msg->points.size(); i++)
        {
            const auto &pt = msg->points[i];

            // Time synchronization
            rclcpp::Time target_time = 
                start_time + rclcpp::Duration(pt.time_from_start);

            // Busy-wait with sleeps to reduce CPU load
            while (this->now() < target_time) {
                rclcpp::sleep_for(std::chrono::milliseconds(1));
            }

            // Convert radians → degrees
            std_msgs::msg::Float32MultiArray out;
            out.data.resize(pt.positions.size());

            for (size_t j = 0; j < pt.positions.size(); j++) {
                out.data[j] = pt.positions[j] * 180.0 / M_PI;
            }

            pub_->publish(out);
        }

        RCLCPP_INFO(this->get_logger(), 
            "Finished sending %zu trajectory points.", msg->points.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItTrajectoryBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    