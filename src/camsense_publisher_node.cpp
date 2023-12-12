#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <camsense_ros2_driver/camsense_x1.h>


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  //RCLCPP_INFO("Starting Camsense_publisher.");
  auto node = std::make_shared<CamsenseX1>("camsense_publisher", rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;
  
  return 0;
}
