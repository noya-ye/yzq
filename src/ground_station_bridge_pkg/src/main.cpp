#include "rclcpp/rclcpp.hpp"
#include "ground_station_bridge_pkg/ground_station_bridge_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ground_station_bridge_pkg::GroundStationBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
