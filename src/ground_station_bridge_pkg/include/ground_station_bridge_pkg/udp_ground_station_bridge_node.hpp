#pragma once

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <ground_station_msgs/msg/task_status.hpp>
#include <ground_station_msgs/msg/ground_command.hpp>
#include <ground_station_msgs/msg/telemetry.hpp>

#include <std_msgs/msg/string.hpp>

#include "ground_station_bridge_pkg/protocol.hpp"

#include <string>
#include <netinet/in.h>

namespace ground_station_bridge_pkg
{

class UdpGroundStationBridgeNode : public rclcpp::Node
{
public:
  UdpGroundStationBridgeNode();
  ~UdpGroundStationBridgeNode() override;

private:
  // ===== callbacks =====
  void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void attitude_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void task_status_cb(const ground_station_msgs::msg::TaskStatus::SharedPtr msg);
  void raw_cmd_cb(const std_msgs::msg::String::SharedPtr msg);

  // ===== timers =====
  void tx_timer_cb();
  void rx_timer_cb();
  void telemetry_pub_timer_cb();

  // ===== helpers =====
  float yaw_from_quat(float q0, float q1, float q2, float q3);
  std::string trim_upper(std::string s) const;

  ground_station_msgs::msg::GroundCommand parse_command(const std::string& raw) const;

  bool udp_init();
  void udp_close();

private:
  // ===== parameters =====
  std::string udp_bind_ip_;
  int udp_bind_port_;

  std::string udp_remote_ip_;
  int udp_remote_port_;

  bool print_tx_;
  bool print_rx_;

  std::string local_pos_topic_;
  std::string attitude_topic_;
  std::string vehicle_status_topic_;
  std::string task_status_topic_;
  std::string raw_cmd_topic_;
  std::string parsed_cmd_topic_;
  std::string telemetry_topic_;

  // ===== udp =====
  int udp_fd_;
  struct sockaddr_in local_addr_;
  struct sockaddr_in remote_addr_;

  // ===== telemetry cache =====
  TelemetryData telemetry_;

  // ===== ros pub/sub =====
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<ground_station_msgs::msg::TaskStatus>::SharedPtr task_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_cmd_sub_;

  rclcpp::Publisher<ground_station_msgs::msg::GroundCommand>::SharedPtr parsed_cmd_pub_;
  rclcpp::Publisher<ground_station_msgs::msg::Telemetry>::SharedPtr telemetry_pub_;

  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_pub_timer_;
};

}  // namespace ground_station_bridge_pkg