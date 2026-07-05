#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

#include "ground_station_msgs/msg/task_status.hpp"
#include "ground_station_msgs/msg/ground_command.hpp"

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/px4_iface.hpp"
#include "offboard_core_pkg/math_tool.hpp"
#include "offboard_core_pkg/tasks.hpp"

using namespace std::chrono_literals;
//积分赛地面站测试程序：用于实现简单的地面站控制飞行器飞行到指定位置并降落，测试地面站与飞控的通信以及飞控的基本飞行控制能力
//目前可能只有开始任务和go_to任务，后续可以根据需要增加其他任务
class OffboardTest4Node : public rclcpp::Node
{
public:
  OffboardTest4Node()
  : Node("offboard_test4_node")
  {
    // ===== params =====
    takeoff_height_m_  = this->declare_parameter<double>("takeoff_height_m", 1.0);
    arrival_error_max_ = this->declare_parameter<double>("arrival_error_max", 0.25);

    wp_xy_tol_ = this->declare_parameter<double>("wp_xy_tol", 0.25);
    wp_z_tol_  = this->declare_parameter<double>("wp_z_tol", 0.25);

    stable_required_ = this->declare_parameter<int>("stable_required", 12);

    hover_s_ = this->declare_parameter<double>("hover_s", 2.0);

    // ===== publishers =====
    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.best_effort();
    qos_pub.durability_volatile();

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_pub);

    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos_pub);

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_pub);

    task_status_pub_ =
      create_publisher<ground_station_msgs::msg::TaskStatus>(
        "/ground_station/task_status", 10);

    // ===== px4 iface =====
    px4_ = std::make_unique<Px4Iface>(
      *this,
      offboard_control_mode_pub_,
      trajectory_setpoint_pub_,
      vehicle_command_pub_);

    // ===== subscriptions =====
    rclcpp::QoS qos_sub(rclcpp::KeepLast(10));
    qos_sub.best_effort();

    vehicle_status_sub_ =
      create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        qos_sub,
        [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
          ctx_.vehicle_status = *msg;
        });

    local_pos_sub_ =
      create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        qos_sub,
        [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          ctx_.local_pos = *msg;

          if (!ctx_.home_inited && ctx_.pos_valid()) {
            ctx_.home_x = ctx_.local_pos.x;
            ctx_.home_y = ctx_.local_pos.y;
            ctx_.home_z = ctx_.local_pos.z;

            ctx_.takeoff_z = static_cast<float>(ctx_.home_z - takeoff_height_m_);
            ctx_.land_z    = ctx_.home_z;

            ctx_.home_inited = true;

            RCLCPP_INFO(
              get_logger(),
              "Home set: x=%.2f y=%.2f z=%.2f takeoff_z=%.2f",
              ctx_.home_x,
              ctx_.home_y,
              ctx_.home_z,
              ctx_.takeoff_z);
          }
        });

    vehicle_land_sub_ =
      create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected",
        qos_sub,
        [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
          ctx_.land_detected = *msg;
        });

    vehicle_att_sub_ =
      create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude",
        qos_sub,
        [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
          ctx_.vehicle_att = *msg;
          ctx_.has_attitude = true;

          if (ctx_.home_inited && !ctx_.home_yaw_inited) {
            const float yaw = math_tool::yaw_from_quat(ctx_.vehicle_att.q);
            if (std::isfinite(yaw)) {
              ctx_.home_yaw = yaw - 0.12f;
              ctx_.home_yaw_inited = true;

              RCLCPP_INFO(
                get_logger(),
                "Home yaw set: %.2f",
                ctx_.home_yaw);
            }
          }
        });

    // ===== ground station command =====
    ground_cmd_sub_ =
      create_subscription<ground_station_msgs::msg::GroundCommand>(
        "/ground_station/cmd_parsed",
        10,
        std::bind(&OffboardTest4Node::ground_cmd_cb, this, std::placeholders::_1));

    // ===== no manual targets =====
    ctx_.detected_targets.clear();
    RCLCPP_WARN(get_logger(), "[OffboardTest4] No manual targets. Waiting for GCS GOTO.");

    // ===== scheduler =====
    build_scheduler();

    timer_ = create_wall_timer(50ms, std::bind(&OffboardTest4Node::on_timer, this));
    last_time_ = now();

    RCLCPP_INFO(get_logger(), "offboard_test4_node started");
  }

private:
  void ground_cmd_cb(const ground_station_msgs::msg::GroundCommand::SharedPtr msg)
  {
    const uint8_t cmd = msg->cmd_type;

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_GOTO) {
      if (!msg->has_goal) {
        RCLCPP_WARN(get_logger(), "[GCS] GOTO received but has_goal=false, ignored");
        return;
      }

      const float gx = msg->x;
      const float gy = msg->y;

      // 地面站发的 z 按“高度”理解：
      // z=1.0 表示离地 1m
      // PX4 local NED 中，向上为负，所以转成 -1.0
      const float gz = -std::fabs(msg->z);

      ctx_.detected_targets.push_back(VisionPosition{gx, gy, gz});

      // 收到新 GOTO 后重建任务链，保证 GoToTask 能读取最新 detected_targets.back()
      ctx_.handover_to_px4_land = false;
      sched_.reset();
      build_scheduler();

      RCLCPP_WARN(
        get_logger(),
        "[GCS] GOTO appended and scheduler rebuilt: rel_x=%.2f rel_y=%.2f rel_z=%.2f | total_targets=%zu",
        gx,
        gy,
        gz,
        ctx_.detected_targets.size());

      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_START) {
      RCLCPP_WARN(get_logger(), "[GCS] START received, mission started");
      mission_started_ = true;
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_PAUSE) {
      RCLCPP_WARN(get_logger(), "[GCS] PAUSE received");
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_RESUME) {
      RCLCPP_WARN(get_logger(), "[GCS] RESUME received");
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_RTL) {
      RCLCPP_WARN(get_logger(), "[GCS] RTL received, ignored in test4");
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_LAND) {
      RCLCPP_WARN(get_logger(), "[GCS] LAND received");
      ctx_.handover_to_px4_land = true;
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_RESET) {
      RCLCPP_WARN(get_logger(), "[GCS] RESET received");

      ctx_.detected_targets.clear();
      ctx_.handover_to_px4_land = false;
      mission_started_ = false;

      sched_.reset();
      build_scheduler();

      RCLCPP_WARN(get_logger(), "[GCS] Scheduler reset. Waiting for GOTO and START.");
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_CLEAR_TRACK) {
      RCLCPP_WARN(get_logger(), "[GCS] CLEAR_TRACK received");
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "[GCS] UNKNOWN cmd_type=%u raw='%s'",
      static_cast<unsigned>(cmd),
      msg->raw.c_str());
  }

  void build_scheduler()
  {
    sched_.reset();

    sched_.add(std::make_unique<WaitHomeTask>(get_logger()));

    sched_.add(std::make_unique<PresetpointTask>(get_logger()));

    sched_.add(std::make_unique<SetOffboardTask>(
      get_logger(),
      *px4_));

    sched_.add(std::make_unique<ArmTask>(
      get_logger(),
      *px4_));

    sched_.add(std::make_unique<TakeoffTask>(
      get_logger(),
      *px4_,
      arrival_error_max_));

    sched_.add(std::make_unique<HoverTask>(
      get_logger(),
      2.0));


    sched_.add(std::make_unique<GoToTask>(
      get_logger(),
      wp_xy_tol_,
      wp_z_tol_,
      stable_required_));

    // GOTO 到达后悬停 hover_s_ 秒
    sched_.add(std::make_unique<HoverTask>(
      get_logger(),
      2.0));

    // 悬停后直接进入 PX4 Land 模式
    sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(
      get_logger(),
      *px4_));
  }

  void publish_task_status()
  {
    ground_station_msgs::msg::TaskStatus msg;
    msg.header.stamp = now();

    msg.task_name = sched_.current_name();
    msg.current_wp = sched_.done() ? sched_.total_count()
                                   : (sched_.current_index() + 1);
    msg.total_wp = sched_.total_count();
    msg.mission_done = sched_.done();

    task_status_pub_->publish(msg);
  }

void on_timer()
{
  const auto t = now();
  const double dt = (t - last_time_).seconds();
  last_time_ = t;

  if (!mission_started_) {
    publish_task_status();
    return;
  }

  // ===== 如果已经进入 PX4 Land 移交，并且已经落地解锁，直接停止任务 =====
  const bool disarmed =
    ctx_.vehicle_status.arming_state ==
    px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;

  if (ctx_.handover_to_px4_land && ctx_.land_detected.landed && disarmed) {
    RCLCPP_WARN(get_logger(), "[MISSION] landed and disarmed, stop mission");
    mission_started_ = false;
    ctx_.handover_to_px4_land = false;
    publish_task_status();
    return;
  }

  if (!sched_.done() && !ctx_.handover_to_px4_land) {
    px4_->publish_offboard_control_mode(ctx_);
  }

  sched_.tick(ctx_, dt);

  // ===== tick 之后再检查一次，防止 LAND task 本轮刚刚完成 =====
  const bool disarmed_after_tick =
    ctx_.vehicle_status.arming_state ==
    px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;

  if (ctx_.handover_to_px4_land && ctx_.land_detected.landed && disarmed_after_tick) {
    RCLCPP_WARN(get_logger(), "[MISSION] landed and disarmed after tick, stop mission");
    mission_started_ = false;
    ctx_.handover_to_px4_land = false;
    publish_task_status();
    return;
  }

  if (sched_.done()) {
    RCLCPP_WARN(get_logger(), "[MISSION] scheduler done, stop mission");
    mission_started_ = false;
    ctx_.handover_to_px4_land = false;
    publish_task_status();
    return;
  }

  if (!ctx_.handover_to_px4_land) {
    px4_->publish_setpoint_from_ctx(ctx_);
  }

  publish_task_status();
}
private:
  Context ctx_;
  std::unique_ptr<Px4Iface> px4_;
  Scheduler sched_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  // ===== parameters =====
  bool mission_started_{false};
  double takeoff_height_m_{1.0};
  double arrival_error_max_{0.25};

  double wp_xy_tol_{0.25};
  double wp_z_tol_{0.25};
  int stable_required_{12};

  double hover_s_{2.0};

  // ===== publishers =====
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<ground_station_msgs::msg::TaskStatus>::SharedPtr task_status_pub_;

  // ===== subscriptions =====
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_att_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_sub_;
  rclcpp::Subscription<ground_station_msgs::msg::GroundCommand>::SharedPtr ground_cmd_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardTest4Node>());
  rclcpp::shutdown();
  return 0;
}
