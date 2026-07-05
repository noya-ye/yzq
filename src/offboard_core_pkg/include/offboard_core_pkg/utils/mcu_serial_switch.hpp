#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace offboard_core_pkg
{

class McuSerialSwitch
{
public:
  McuSerialSwitch(
      rclcpp::Logger logger,
      std::string port = "/dev/ttyUSB0",
      int baudrate = 115200,
      bool enabled = true);

  ~McuSerialSwitch();

  bool open();
  void close();

  bool is_open() const;
  bool is_on() const;

  bool set(bool on);
  bool on();
  bool off();

private:
  bool configure_port_();

private:
  rclcpp::Logger logger_;
  std::string port_;
  int baudrate_{115200};
  bool enabled_{true};

  int fd_{-1};
  bool switch_on_{false};
};

}  // namespace offboard_core_pkg