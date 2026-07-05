#include "ground_station_bridge_pkg/protocol.hpp"

#include <iomanip>
#include <sstream>

namespace ground_station_bridge_pkg
{

uint8_t Protocol::xor_checksum(const std::string& text)
{
  uint8_t cs = 0;
  for (const auto c : text) {
    cs ^= static_cast<uint8_t>(c);
  }
  return cs;
}

std::string Protocol::encode_status(const TelemetryData& data)
{
  std::ostringstream body;
  body << "STAT"
       << ",lp=" << (data.local_pos_valid ? 1 : 0)
       << ",att=" << (data.attitude_valid ? 1 : 0)
       << ",st=" << (data.status_valid ? 1 : 0)
       << ",arm=" << static_cast<int>(data.arming_state)
       << ",nav=" << static_cast<int>(data.nav_state)
       << ",x=" << std::fixed << std::setprecision(2) << data.x
       << ",y=" << std::fixed << std::setprecision(2) << data.y
       << ",z=" << std::fixed << std::setprecision(2) << data.z
       << ",vx=" << std::fixed << std::setprecision(2) << data.vx
       << ",vy=" << std::fixed << std::setprecision(2) << data.vy
       << ",vz=" << std::fixed << std::setprecision(2) << data.vz
       << ",yaw=" << std::fixed << std::setprecision(2) << data.yaw
       << ",task=" << data.task_name
       << ",wp=" << data.current_wp << "/" << data.total_wp
       << ",done=" << (data.mission_done ? 1 : 0)
       << ",type=" << data.target_type
       << ",target=" << data.target_name;

  const std::string payload = body.str();
  const uint8_t cs = xor_checksum(payload);

  std::ostringstream out;
  out << "$" << payload << "*"
      << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(cs)
      << "\r\n";

  return out.str();
}

}  // namespace ground_station_bridge_pkg
