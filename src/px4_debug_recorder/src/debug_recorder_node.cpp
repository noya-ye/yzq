#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

struct SampleRow
{
  double t_sec{0.0};

  float x{0.f}, y{0.f}, z{0.f};
  float vx{0.f}, vy{0.f}, vz{0.f};

  float yaw{0.f};
  float yaw_rate{0.f};

  float sp_x{0.f}, sp_y{0.f}, sp_z{0.f};
  float sp_vx{0.f}, sp_vy{0.f}, sp_vz{0.f};
  float sp_yaw{0.f};

  bool offboard_pos_ctrl{false};
  bool offboard_vel_ctrl{false};

  uint8_t arming_state{0};
  uint8_t nav_state{0};

  bool xy_valid{false};
  bool z_valid{false};
  bool landed{false};
};

class DebugRecorderNode : public rclcpp::Node
{
public:
  DebugRecorderNode() : Node("debug_recorder_node")
  {
    sample_rate_hz_ = declare_parameter<double>("sample_rate_hz", 10.0);
    max_rows_ = declare_parameter<int>("max_rows", 200000);
    log_root_dir_ = declare_parameter<std::string>(
      "log_root_dir",
      std::string(std::getenv("HOME")) + "/debug_logs");

    // 录制触发条件
    record_when_armed_ = declare_parameter<bool>("record_when_armed", true);
    record_when_offboard_ = declare_parameter<bool>("record_when_offboard", true);
    auto_stop_on_disarm_ = declare_parameter<bool>("auto_stop_on_disarm", false);

    vehicle_status_topic_ = declare_parameter<std::string>(
      "vehicle_status_topic", "/fmu/out/vehicle_status_v1");
    local_position_topic_ = declare_parameter<std::string>(
      "local_position_topic", "/fmu/out/vehicle_local_position");
    vehicle_attitude_topic_ = declare_parameter<std::string>(
      "vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
    land_detected_topic_ = declare_parameter<std::string>(
      "land_detected_topic", "/fmu/out/vehicle_land_detected");
    offboard_mode_topic_ = declare_parameter<std::string>(
      "offboard_mode_topic", "/fmu/in/offboard_control_mode");
    trajectory_setpoint_topic_ = declare_parameter<std::string>(
      "trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");

    rows_.reserve(static_cast<size_t>(std::max(1000, max_rows_)));

    setup_output_dir();
    setup_subscribers();
    setup_timer();

    RCLCPP_INFO(get_logger(), "PX4 Debug Recorder started");
    RCLCPP_INFO(get_logger(), "sample_rate_hz = %.2f", sample_rate_hz_);
    RCLCPP_INFO(get_logger(), "log_dir = %s", session_dir_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "record_when_armed=%d, record_when_offboard=%d, auto_stop_on_disarm=%d",
      static_cast<int>(record_when_armed_),
      static_cast<int>(record_when_offboard_),
      static_cast<int>(auto_stop_on_disarm_));
  }

  ~DebugRecorderNode() override
  {
    save_all();
  }

private:
  void setup_output_dir()
  {
    const auto now_sys = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now_sys);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);

    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S");

    session_dir_ = log_root_dir_ + "/" + oss.str();
    fs::create_directories(session_dir_);

    csv_path_ = session_dir_ + "/state_log.csv";
    meta_path_ = session_dir_ + "/meta.txt";
  }

  void setup_subscribers()
  {
    rclcpp::QoS qos_out(rclcpp::KeepLast(10));
    qos_out.best_effort();
    qos_out.durability_volatile();

    sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic_, qos_out,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        status_ = *msg;
        has_status_ = true;
      });

    sub_local_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      local_position_topic_, qos_out,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        local_pos_ = *msg;
        has_local_pos_ = true;
      });

    sub_att_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      vehicle_attitude_topic_, qos_out,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        att_ = *msg;
        has_att_ = true;
      });

    sub_land_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      land_detected_topic_, qos_out,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        land_ = *msg;
        has_land_ = true;
      });

    sub_offboard_ = create_subscription<px4_msgs::msg::OffboardControlMode>(
      offboard_mode_topic_, 10,
      [this](const px4_msgs::msg::OffboardControlMode::SharedPtr msg) {
        offboard_mode_ = *msg;
        has_offboard_mode_ = true;
      });

    sub_sp_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      trajectory_setpoint_topic_, 10,
      [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        sp_ = *msg;
        has_sp_ = true;
      });
  }

  void setup_timer()
  {
    if (sample_rate_hz_ <= 0.0) {
      sample_rate_hz_ = 10.0;
    }

    const auto period_ms = static_cast<int>(1000.0 / sample_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, period_ms)),
      std::bind(&DebugRecorderNode::on_timer, this));
  }

  static float yaw_from_quat(const px4_msgs::msg::VehicleAttitude & att)
  {
    const float q0 = att.q[0];
    const float q1 = att.q[1];
    const float q2 = att.q[2];
    const float q3 = att.q[3];

    const float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    const float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  bool is_armed() const
  {
    if (!has_status_) {
      return false;
    }
    return status_.arming_state ==
      px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool is_offboard() const
  {
    if (!has_status_) {
      return false;
    }
    return status_.nav_state ==
      px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool should_record_now() const
  {
    const bool armed_ok = record_when_armed_ && is_armed();
    const bool offboard_ok = record_when_offboard_ && is_offboard();
    return armed_ok || offboard_ok;
  }

  void on_timer()
  {
    const bool should_record = should_record_now();

    if (!recording_started_ && should_record) {
      recording_started_ = true;
      record_start_time_sec_ = now().seconds();
      RCLCPP_INFO(
        get_logger(),
        "Recording triggered: armed=%d offboard=%d",
        static_cast<int>(is_armed()),
        static_cast<int>(is_offboard()));
    }

    if (auto_stop_on_disarm_ && recording_started_ && !should_record) {
      RCLCPP_INFO(get_logger(), "Recording auto-stopped: trigger condition no longer met");
      save_all();
      rclcpp::shutdown();
      return;
    }

    if (!recording_started_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for trigger... armed=%d offboard=%d",
        static_cast<int>(is_armed()),
        static_cast<int>(is_offboard()));
      return;
    }

    if (static_cast<int>(rows_.size()) >= max_rows_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Max rows reached (%d), stop sampling more data", max_rows_);
      return;
    }

    SampleRow row;
    row.t_sec = now().seconds();

    if (has_local_pos_) {
      row.x = local_pos_.x;
      row.y = local_pos_.y;
      row.z = local_pos_.z;
      row.vx = local_pos_.vx;
      row.vy = local_pos_.vy;
      row.vz = local_pos_.vz;
      row.xy_valid = local_pos_.xy_valid;
      row.z_valid = local_pos_.z_valid;
    }

    if (has_att_) {
      row.yaw = yaw_from_quat(att_);
      row.yaw_rate = 0.0f;
    }

    if (has_sp_) {
      row.sp_x = sp_.position[0];
      row.sp_y = sp_.position[1];
      row.sp_z = sp_.position[2];

      row.sp_vx = sp_.velocity[0];
      row.sp_vy = sp_.velocity[1];
      row.sp_vz = sp_.velocity[2];

      row.sp_yaw = sp_.yaw;
    }

    if (has_offboard_mode_) {
      row.offboard_pos_ctrl = offboard_mode_.position;
      row.offboard_vel_ctrl = offboard_mode_.velocity;
    }

    if (has_status_) {
      row.arming_state = status_.arming_state;
      row.nav_state = status_.nav_state;
    }

    if (has_land_) {
      row.landed = land_.landed;
    }

    rows_.push_back(row);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Recording... samples=%zu armed=%d offboard=%d pos=(%.2f %.2f %.2f) sp=(%.2f %.2f %.2f) yaw=%.2f",
      rows_.size(),
      static_cast<int>(is_armed()),
      static_cast<int>(is_offboard()),
      row.x, row.y, row.z,
      row.sp_x, row.sp_y, row.sp_z,
      row.yaw);
  }

  void save_all()
  {
    if (saved_) {
      return;
    }
    saved_ = true;

    if (!recording_started_ || rows_.empty()) {
      RCLCPP_WARN(get_logger(), "No flight data recorded, skip saving CSV");
      save_meta_only();
      return;
    }

    save_csv();
    save_meta();
  }

  void save_csv()
  {
    std::ofstream ofs(csv_path_, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
      return;
    }

    ofs << "t_sec,"
        << "x,y,z,"
        << "vx,vy,vz,"
        << "yaw,yaw_rate,"
        << "sp_x,sp_y,sp_z,"
        << "sp_vx,sp_vy,sp_vz,"
        << "sp_yaw,"
        << "offboard_pos_ctrl,offboard_vel_ctrl,"
        << "arming_state,nav_state,"
        << "xy_valid,z_valid,landed\n";

    for (const auto & r : rows_) {
      ofs << r.t_sec << ","
          << r.x << "," << r.y << "," << r.z << ","
          << r.vx << "," << r.vy << "," << r.vz << ","
          << r.yaw << "," << r.yaw_rate << ","
          << r.sp_x << "," << r.sp_y << "," << r.sp_z << ","
          << r.sp_vx << "," << r.sp_vy << "," << r.sp_vz << ","
          << r.sp_yaw << ","
          << static_cast<int>(r.offboard_pos_ctrl) << ","
          << static_cast<int>(r.offboard_vel_ctrl) << ","
          << static_cast<int>(r.arming_state) << ","
          << static_cast<int>(r.nav_state) << ","
          << static_cast<int>(r.xy_valid) << ","
          << static_cast<int>(r.z_valid) << ","
          << static_cast<int>(r.landed)
          << "\n";
    }

    ofs.close();
    RCLCPP_INFO(get_logger(), "Saved CSV: %s", csv_path_.c_str());
  }

  void save_meta_only()
  {
    std::ofstream ofs(meta_path_, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
      return;
    }

    ofs << "PX4 Debug Recorder Meta\n";
    ofs << "=======================\n";
    ofs << "recording_started: " << recording_started_ << "\n";
    ofs << "samples: " << rows_.size() << "\n";
    ofs << "note: no triggered flight data recorded\n";
    ofs.close();

    RCLCPP_INFO(get_logger(), "Saved meta only: %s", meta_path_.c_str());
  }

  void save_meta()
  {
    std::ofstream ofs(meta_path_, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open meta file: %s", meta_path_.c_str());
      return;
    }

    ofs << "PX4 Debug Recorder Meta\n";
    ofs << "=======================\n";
    ofs << "sample_rate_hz: " << sample_rate_hz_ << "\n";
    ofs << "samples: " << rows_.size() << "\n";
    ofs << "log_dir: " << session_dir_ << "\n";
    ofs << "record_when_armed: " << record_when_armed_ << "\n";
    ofs << "record_when_offboard: " << record_when_offboard_ << "\n";
    ofs << "auto_stop_on_disarm: " << auto_stop_on_disarm_ << "\n";
    ofs << "recording_started: " << recording_started_ << "\n";
    ofs << "record_start_time_sec: " << record_start_time_sec_ << "\n\n";

    ofs << "Topics:\n";
    ofs << "  vehicle_status_topic: " << vehicle_status_topic_ << "\n";
    ofs << "  local_position_topic: " << local_position_topic_ << "\n";
    ofs << "  vehicle_attitude_topic: " << vehicle_attitude_topic_ << "\n";
    ofs << "  land_detected_topic: " << land_detected_topic_ << "\n";
    ofs << "  offboard_mode_topic: " << offboard_mode_topic_ << "\n";
    ofs << "  trajectory_setpoint_topic: " << trajectory_setpoint_topic_ << "\n\n";

    ofs << "Received flags:\n";
    ofs << "  has_status: " << has_status_ << "\n";
    ofs << "  has_local_pos: " << has_local_pos_ << "\n";
    ofs << "  has_att: " << has_att_ << "\n";
    ofs << "  has_land: " << has_land_ << "\n";
    ofs << "  has_offboard_mode: " << has_offboard_mode_ << "\n";
    ofs << "  has_sp: " << has_sp_ << "\n";

    ofs.close();
    RCLCPP_INFO(get_logger(), "Saved meta: %s", meta_path_.c_str());
  }

private:
  double sample_rate_hz_{10.0};
  int max_rows_{200000};

  bool record_when_armed_{true};
  bool record_when_offboard_{true};
  bool auto_stop_on_disarm_{false};

  std::string log_root_dir_;
  std::string session_dir_;
  std::string csv_path_;
  std::string meta_path_;

  std::string vehicle_status_topic_;
  std::string local_position_topic_;
  std::string vehicle_attitude_topic_;
  std::string land_detected_topic_;
  std::string offboard_mode_topic_;
  std::string trajectory_setpoint_topic_;

  px4_msgs::msg::VehicleStatus status_{};
  px4_msgs::msg::VehicleLocalPosition local_pos_{};
  px4_msgs::msg::VehicleAttitude att_{};
  px4_msgs::msg::VehicleLandDetected land_{};
  px4_msgs::msg::OffboardControlMode offboard_mode_{};
  px4_msgs::msg::TrajectorySetpoint sp_{};

  bool has_status_{false};
  bool has_local_pos_{false};
  bool has_att_{false};
  bool has_land_{false};
  bool has_offboard_mode_{false};
  bool has_sp_{false};

  bool recording_started_{false};
  bool saved_{false};
  double record_start_time_sec_{0.0};

  std::vector<SampleRow> rows_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_att_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr sub_land_;
  rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr sub_offboard_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_sp_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DebugRecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
