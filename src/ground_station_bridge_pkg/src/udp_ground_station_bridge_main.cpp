#include "ground_station_bridge_pkg/udp_ground_station_bridge_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ground_station_bridge_pkg::UdpGroundStationBridgeNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}