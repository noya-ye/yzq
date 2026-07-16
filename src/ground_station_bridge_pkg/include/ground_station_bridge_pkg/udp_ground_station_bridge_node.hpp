#pragma once

#include <netinet/in.h>

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ground_station_msgs/msg/animal_info.hpp"
#include "ground_station_msgs/msg/ground_command.hpp"
#include "ground_station_msgs/msg/mission_plan.hpp"
#include "ground_station_msgs/msg/mission_result.hpp"
#include "ground_station_msgs/msg/task_status.hpp"

namespace ground_station_bridge_pkg
{

class UdpGroundStationBridgeNode : public rclcpp::Node
{
public:
  UdpGroundStationBridgeNode();
  ~UdpGroundStationBridgeNode() override;

private:
  bool udp_init();
  void udp_close();
  void send_udp(const std::string& data);

  void raw_cmd_cb(const std_msgs::msg::String::SharedPtr msg);
  void rx_timer_cb();

  void mission_plan_cb(
    const ground_station_msgs::msg::MissionPlan::SharedPtr msg);

  void task_status_cb(
    const ground_station_msgs::msg::TaskStatus::SharedPtr msg);

  void animal_info_cb(
    const ground_station_msgs::msg::AnimalInfo::SharedPtr msg);

  void mission_result_cb(
    const ground_station_msgs::msg::MissionResult::SharedPtr msg);

  ground_station_msgs::msg::GroundCommand parse_command(
    const std::string& raw) const;

  static std::string trim(const std::string& text);
  static std::string upper(std::string text);
  static std::string escape_json(const std::string& text);

  static std::string json_string(
    const std::string& json,
    const std::string& key);

  static std::vector<std::string> json_string_array(
    const std::string& json,
    const std::string& key);

private:
  std::string udp_bind_ip_;
  int udp_bind_port_{9000};

  std::string udp_remote_ip_;
  int udp_remote_port_{9001};

  bool print_tx_{true};
  bool print_rx_{true};

  std::string raw_cmd_topic_;
  std::string parsed_cmd_topic_;
  std::string mission_plan_topic_;
  std::string task_status_topic_;
  std::string animal_info_topic_;
  std::string mission_result_topic_;

  int udp_fd_{-1};

  sockaddr_in local_addr_{};
  sockaddr_in remote_addr_{};

  rclcpp::Publisher<
    ground_station_msgs::msg::GroundCommand>::SharedPtr
    parsed_cmd_pub_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    raw_cmd_sub_;

  rclcpp::Subscription<
    ground_station_msgs::msg::MissionPlan>::SharedPtr
    mission_plan_sub_;

  rclcpp::Subscription<
    ground_station_msgs::msg::TaskStatus>::SharedPtr
    task_status_sub_;

  rclcpp::Subscription<
    ground_station_msgs::msg::AnimalInfo>::SharedPtr
    animal_info_sub_;

  rclcpp::Subscription<
    ground_station_msgs::msg::MissionResult>::SharedPtr
    mission_result_sub_;

  rclcpp::TimerBase::SharedPtr rx_timer_;
};

}  // namespace ground_station_bridge_pkg