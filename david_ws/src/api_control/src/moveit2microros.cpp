#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>

class MoveitMirrorBridge : public rclcpp::Node
{
public:
    MoveitMirrorBridge() : Node("movet2microros")
    {
        target_joint_names_ = {
            "joint1","joint2","joint3",
            "joint4","joint5","joint6"
        };

        pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/esp_joint_states", 10);

        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MoveitMirrorBridge::callback, this, std::placeholders::_1));

        // --- 50 Hz timer ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),       //////////////////////////////////////////
            std::bind(&MoveitMirrorBridge::publish_filtered, this));

        RCLCPP_INFO(this->get_logger(), "MoveIt Mirror Bridge initialized.");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::JointState last_msg_;
    bool have_msg_ = false;

    std::vector<std::string> target_joint_names_;

    void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_msg_ = *msg;     // store latest msg
        have_msg_ = true;
    }

    void publish_filtered()
    {
        if (!have_msg_) return;

        const auto &msg = last_msg_;
        sensor_msgs::msg::JointState filtered;

        // Use current time to avoid micro-ROS timestamp issues
        filtered.header.stamp = this->now();

        // Build name->index map
        std::unordered_map<std::string, size_t> index_map;
        for (size_t i = 0; i < msg.name.size(); i++)
            index_map[msg.name[i]] = i;

        // Extract relevant joints
        for (const auto &joint : target_joint_names_)
        {
            filtered.name.push_back(joint);

            if (index_map.count(joint) && index_map[joint] < msg.position.size())
                filtered.position.push_back(msg.position[index_map[joint]]);
            else
                filtered.position.push_back(0.0);
        }

        pub_->publish(filtered);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveitMirrorBridge>());
    rclcpp::shutdown();
    return 0;
}
