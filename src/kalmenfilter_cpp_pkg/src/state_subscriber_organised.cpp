
#include "state_subscriber.hpp"

SubscriberNode::SubscriberNode() : Node("subscriber")
{
  //      topic name        queue size
  IMU_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/rexrov/imu", 100,
                                                                     //                 callback function           number of parameters
                                                                     std::bind(&FilterProcessingUnit::callbackIMU, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "subscriber IMU initialized");

  DVL_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/rexrov/dvl_twist", 100,
                                                                                              std::bind(&FilterProcessingUnit::callbackDVL, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "subscriber DVL initialized");

  Pressure_subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>("/rexrov/pressure", 100,

                                                                                    std::bind(&FilterProcessingUnit::callbackPressure, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "subscriber Pressure initialized");

  Truth_subscriber_ = this->create_subscription<gazebo_msgs::msg::LinkStates>("/gazebo/link_states", 100,

                                                                              std::bind(&FilterProcessingUnit::callbackGroundTruth, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "subscriber ground truth initialized");
  }


  

 

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
