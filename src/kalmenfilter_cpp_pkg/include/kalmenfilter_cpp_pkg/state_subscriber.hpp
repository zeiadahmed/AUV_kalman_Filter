#ifndef STATE_SUBSCRIBER_HPP_
#define STATE_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "filter_preocessing_unit.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode();
private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMU_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr DVL_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr Pressure_subscriber_;
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr Truth_subscriber_;
};
#endif
