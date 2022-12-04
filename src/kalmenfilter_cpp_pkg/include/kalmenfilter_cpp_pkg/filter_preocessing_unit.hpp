#ifndef FILTER_PROCESSING_UNIT_HPP_
#define FILTER_PROCESSING_UNIT_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

class FilterProcessingUnit :public rclcpp::Node
{
  public:
    FilterProcessingUnit();
    void callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg);
    void callbackDVL(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void callbackPressure(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
    void callbackGroundTruth(const gazebo_msgs::msg::LinkStates::SharedPtr msg);
    
};
#endif
