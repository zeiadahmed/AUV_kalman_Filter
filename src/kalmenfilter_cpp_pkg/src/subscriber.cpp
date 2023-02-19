#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("subscriber")
  {
    //                                                              topic name        queue size
    subscriber_ = this->create_subscription<std_msgs::msg::String>("news", 10,
                                                                   //                                                               callback function                       number of parameters
                                                                   std::bind(&SubscriberNode::callbackNews, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),"subscriber initialized");
  }

private:
  void callbackNews(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
