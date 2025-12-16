#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class MoveitMirrorBridge : public rclcpp::Node
{
public:
    MoveitMirrorBridge(): Node("movet2microros2")
    {
        target_joint_names_ = {
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        };
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/esp_joint_states", 10);

        sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,std::bind(&MoveitMirrorBridge::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MoveIt Mirror Bridge initialized. Subscribing to /joint_states");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    std::vector<std::string> target_joint_names_;

    void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
     // Build name -> index map for speed
        std::unordered_map<std::string, size_t> index_map;
        for (size_t i = 0; i < msg->name.size(); i++)
            index_map[msg->name[i]] = i;

        sensor_msgs::msg::JointState filtered;
        filtered.header.stamp = msg->header.stamp;

        for (const auto &joint : target_joint_names_)
        {
            if (index_map.count(joint))
            {
                size_t idx = index_map[joint];
                filtered.name.push_back(joint);

                if (msg->position.size() > idx){
                    filtered.position.push_back(msg->position[idx]);}
                else{
                    filtered.position.push_back(0.0);}// default fallback
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Joint '%s' not found in /joint_states", joint.c_str());
            }
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
