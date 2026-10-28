#include "drone_mission/mission_node.hpp"

int main(int argc, char ** argv)
{
  // Initialize ROS before constructing nodes, then keep the node alive in the executor.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
