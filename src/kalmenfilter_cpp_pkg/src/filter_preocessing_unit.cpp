#include "filter_preocessing_unit.hpp"

FilterProcessingUnit::FilterProcessingUnit() :Node("dummy"){}

void FilterProcessingUnit::callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  
  // RCLCPP_INFO(this->get_logger(), "Imu  seq: [%d]", msg->header.frame_id);
  // RCLCPP_INFO(this->get_logger(), "Imu  Oriantation x:[%f], y: [%f], z: [%f] ,w: [%f]",
  //             msg->orientation.x,
  //             msg->orientation.y,
  //             msg->orientation.z,
  //             msg->orientation.w);
}
void FilterProcessingUnit::callbackDVL(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "DVL linear velocity in x: [%f], y: [%f], z: [%f]",
  //             msg->twist.twist.linear.x,
  //             msg->twist.twist.linear.y,
  //             msg->twist.twist.linear.z);
}
void FilterProcessingUnit::callbackPressure(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{

  // RCLCPP_INFO(this->get_logger(), "pressure reading :[%f]" , msg->fluid_pressure);
  // RCLCPP_INFO(this->get_logger(), "pressure variance:[%f]" , msg->variance);
}

void FilterProcessingUnit::callbackGroundTruth(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
{

  // RCLCPP_INFO(this->get_logger(), "position in x:[%f], y: [%f], z: [%f] ",
  //             msg->pose.data()->position.x,
  //             msg->pose.data()->position.y,
  //             msg->pose.data()->position.z);

  // RCLCPP_INFO(this->get_logger(), "oriantation in x:[%f], y: [%f], z: [%f] ",
  //             msg->pose.data()->orientation.x,
  //             msg->pose.data()->orientation.y,
  //             msg->pose.data()->orientation.z);

  // RCLCPP_INFO(this->get_logger(), "twist linear in x:[%f], y: [%f], z: [%f] ",
  //             msg->twist.data()->linear.x,
  //             msg->twist.data()->linear.y,
  //             msg->twist.data()->linear.z);

  // RCLCPP_INFO(this->get_logger(), "twist angular in x:[%f], y: [%f], z: [%f] ",
  //             msg->twist.data()->angular.x,
  //             msg->twist.data()->angular.y,
  //             msg->twist.data()->angular.z);
}
