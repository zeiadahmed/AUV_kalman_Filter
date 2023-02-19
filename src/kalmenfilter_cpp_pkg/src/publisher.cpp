#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherNode : public rclcpp::Node // MODIFY NAME
{
public:
    PublisherNode() : Node("news_publisher"), robot_name_("R2D2") // MODIFY NAME
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("news", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&PublisherNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(),"publisher started");
        
    }

private:
    void publishNews()
    {
        auto msg = std_msgs::msg::String();
        msg.data = std::string("Hi , this is ") + robot_name_ + std::string(" from publisher");
        publisher_->publish(msg);
    }

    std::string robot_name_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
