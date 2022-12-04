cmake_minimum_required(VERSION 3.5)
project(kalmenfilter_cpp_pkg)



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
# add_executable(cpp_node src/template_cpp_node.cpp)
# ament_target_dependencies(cpp_node rclcpp)

# add_executable(news_publisher src/publisher.cpp)
# ament_target_dependencies(news_publisher rclcpp std_msgs)

# add_executable(subscriber src/subscriber.cpp)
# ament_target_dependencies(subscriber rclcpp std_msgs)
 
add_executable(state_subscriber src/state_subscriber.cpp )
ament_target_dependencies(state_subscriber rclcpp sensor_msgs geometry_msgs gazebo_msgs nav_msgs tf2)


MESSAGE( [Main] " EIGEN3_INCLUDE_DIRS = ${EIGEN3_INCLUDE_DIRS}") 
# EIGEN3_INCLUDE_DIRS = /usr/include/eigen3

include_directories(${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  # cpp_node
  # news_publisher
  # subscriber
  state_subscriber
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
