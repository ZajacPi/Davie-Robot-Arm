#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class TestEspPublisher : public rclcpp::Node
{
public:
    TestEspPublisher() : Node("test_publisher_node")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/esp_joint_states", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TestEspPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Test ESP publisher started.");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double t_ = 0.0;

    void timer_callback()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"joint1","joint2","joint3","joint4","joint5","joint6"};
        
        // example: oscillate joint2
        double angle = 0.5 * sin(t_); // radians
        msg.position = {0.0, angle, 0.0, 0.0, 0.0, 0.0};

        pub_->publish(msg);
        t_ += 0.1;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestEspPublisher>());
    rclcpp::shutdown();
    return 0;
}
