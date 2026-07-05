#pragma once

#include <cstdint>
#include <string>

namespace ground_station_bridge_pkg
{

struct TelemetryData
{
  bool local_pos_valid{false};
  bool attitude_valid{false};
  bool status_valid{false};

  float x{0.0f};
  float y{0.0f};
  float z{0.0f};

  float vx{0.0f};
  float vy{0.0f};
  float vz{0.0f};

  float yaw{0.0f};

  uint8_t arming_state{0};
  uint8_t nav_state{0};

  std::string task_name{"IDLE"};
  int current_wp{0};
  int total_wp{0};
  bool mission_done{false};
  int32_t target_type{0};
  std::string target_name{"none"};

  uint64_t stamp_us{0};
};

}  // namespace ground_station_bridge_pkg
