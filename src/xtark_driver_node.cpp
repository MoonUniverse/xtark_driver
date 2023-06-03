#include "xtark_driver.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions xtark_driver_options;
  xtark_driver_options.arguments(
      {"--ros-args"});
  auto xtark_driver = std::make_shared<xtark_driver::XtarkDriver>(xtark_driver_options);
  exec.add_node(xtark_driver);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}