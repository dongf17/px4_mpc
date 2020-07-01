#include <rclcpp/rclcpp.hpp>

#include "px4_mpc/offboard_control.hpp"


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "Starting odometry listener..." << std::endl;
  auto n = rclcpp::Node::make_shared("mpc_runner");
  OffboardControl offboard_controller(n);
  
  rclcpp::spin(n);
  rclcpp::shutdown();

  return 0;
}
