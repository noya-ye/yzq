#pragma once

#include <string>

namespace ground_station_bridge_pkg
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  bool open_port(const std::string& device, int baudrate);
  void close_port();
  bool is_open() const;

  bool write_data(const std::string& data);
  int read_data(char* buffer, int max_len);

private:
  int fd_{-1};

  int baudrate_to_flag(int baudrate);
};

}  // namespace ground_station_bridge_pkg
