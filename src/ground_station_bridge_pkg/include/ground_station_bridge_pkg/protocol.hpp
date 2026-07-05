#pragma once

#include <string>
#include "ground_station_bridge_pkg/telemetry_data.hpp"

namespace ground_station_bridge_pkg
{

class Protocol
{
public:
  static std::string encode_status(const TelemetryData& data);
  static uint8_t xor_checksum(const std::string& text);
};

}  // namespace ground_station_bridge_pkg
