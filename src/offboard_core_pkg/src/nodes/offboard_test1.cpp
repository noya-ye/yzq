// 自转扫描测试节点：
// 起飞 -> 悬停 -> YawSpin 开启前视视觉 -> 扫描一圈 -> 悬停 -> PX4 Land
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "std_msgs/msg/bool.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/px4_iface.hpp"
#include "offboard_core_pkg/math_tool.hpp"
#include "offboard_core_pkg/tasks.hpp"
#include "offboard_core_pkg/tasks/yaw_spin_task.hpp"

#include "custom_vision_msgs/msg/position_target_array.hpp"

using namespace std::chrono_literals;

class OffboardTestNode : public rclcpp::Node
{
public:
  OffboardTestNode()
  : Node("offboard_test1_node")
  {
    // ===== params =====
    takeoff_height_m_ =
      this->declare_parameter<double>("takeoff_height_m", 1.0);

    arrival_error_max_ =
      this->declare_parameter<double>("arrival_error_max", 0.25);

    hover_s_ =
      this->declare_parameter<double>("hover_s", 2.0);

    yaw_rate_ =
      this->declare_parameter<double>("yaw_rate", 0.25);   // rad/s，先保守

    front_stabilize_s_ =
      this->declare_parameter<double>("front_stabilize_s", 0.5);

    max_target_xy_m_ =
      this->declare_parameter<double>("max_target_xy_m", 4.0);

    min_target_z_m_ =
      this->declare_parameter<double>("min_target_z_m", -2.0);

    max_target_z_m_ =
      this->declare_parameter<double>("max_target_z_m", 0.5);

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

    vision_front_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(
        "/vision/front/enable", 10);

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

            // PX4 local NED：向上是负 z
            ctx_.takeoff_z =
              static_cast<float>(ctx_.home_z - takeoff_height_m_);

            ctx_.land_z = ctx_.home_z;
            ctx_.home_inited = true;

            RCLCPP_INFO(
              get_logger(),
              "[HOME] set: x=%.2f y=%.2f z=%.2f takeoff_z=%.2f",
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

          const float yaw = math_tool::yaw_from_quat(ctx_.vehicle_att.q);
          if (std::isfinite(yaw)) {
            ctx_.yaw = yaw;
          }

          if (ctx_.home_inited && !ctx_.home_yaw_inited) {
            if (std::isfinite(yaw)) {
              ctx_.home_yaw = yaw;
              ctx_.home_yaw_inited = true;

              RCLCPP_INFO(
                get_logger(),
                "[HOME] yaw set: %.2f",
                ctx_.home_yaw);
            }
          }
        });

    targets_sub_ =
      create_subscription<custom_vision_msgs::msg::PositionTargetArray>(
        "/vision/position_targets",
        qos_sub,
        std::bind(&OffboardTestNode::targets_cb, this, std::placeholders::_1));

    ctx_.detected_targets.clear();

    build_scheduler();

    timer_ = create_wall_timer(
      50ms,
      std::bind(&OffboardTestNode::on_timer, this));

    last_time_ = now();

    RCLCPP_WARN(
      get_logger(),
      "[OffboardTest1] started | takeoff_height=%.2f | yaw_rate=%.2f rad/s | front_stabilize=%.2fs",
      takeoff_height_m_,
      yaw_rate_,
      front_stabilize_s_);
  }

private:
  bool target_valid(float x, float y, float z) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }

    const float dxy = std::sqrt(x * x + y * y);
    if (dxy > max_target_xy_m_) {
      return false;
    }

    if (z < min_target_z_m_ || z > max_target_z_m_) {
      return false;
    }

    return true;
  }

  bool is_duplicate_target(float x, float y) const
  {
    for (const auto& p : ctx_.detected_targets) {
      const float dx = p.x - x;
      const float dy = p.y - y;
      const float d = std::sqrt(dx * dx + dy * dy);

      if (d < 0.30f) {
        return true;
      }
    }

    return false;
  }

  void targets_cb(
    const custom_vision_msgs::msg::PositionTargetArray::SharedPtr msg)
  {
    // 只有 YawSpinTask 打开前视门控时，才接收视觉目标
    if (!ctx_.vision_front_enable) {
      return;
    }

    const bool already_stabilizing = front_stabilizing_;

    for (const auto& t : msg->targets) {
      const float x = t.x;
      const float y = t.y;
      const float z = t.z;

      if (!target_valid(x, y, z)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[VISION_FRONT] invalid target ignored: x=%.2f y=%.2f z=%.2f",
          x,
          y,
          z);
        continue;
      }

      if (is_duplicate_target(x, y)) {
        continue;
      }

      ctx_.detected_targets.push_back(VisionPosition{x, y, z});

      RCLCPP_WARN(
        get_logger(),
        "[VISION_FRONT] append target: x=%.2f y=%.2f z=%.2f total=%zu",
        x,
        y,
        z,
        ctx_.detected_targets.size());

      // 只在“不处于稳定采样阶段”时触发 YawSpin 暂停。
      // 稳定采样期间如果收到异常新点，可以记录，但不刷新 front_stabilize_start_。
      if (!already_stabilizing) {
        ctx_.vision_detected = true;
        ctx_.vision_done = false;

        front_stabilizing_ = true;
        front_stabilize_start_ = this->now();

        RCLCPP_WARN(
          get_logger(),
          "[VISION_FRONT] pause yaw scan for %.2fs",
          front_stabilize_s_);
      }
    }
  }

  void build_scheduler()
  {
    sched_.clear();

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
      hover_s_));

    sched_.add(std::make_unique<offboard_core_pkg::YawSpinTask>(
      get_logger(),
      yaw_rate_));

    sched_.add(std::make_unique<HoverTask>(
      get_logger(),
      hover_s_));

    sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(
      get_logger(),
      *px4_));
    sched_.reset();
  }

  void publish_vision_gate()
  {
    std_msgs::msg::Bool msg;
    msg.data = ctx_.vision_front_enable;
    vision_front_enable_pub_->publish(msg);
  }

  void publish_land_command()
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;

    cmd.param1 = 0.0f;
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;

    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    vehicle_command_pub_->publish(cmd);
  }

  void request_land(const std::string& reason)
  {
    if (!ctx_.handover_to_px4_land) {
      RCLCPP_ERROR(
        get_logger(),
        "[SAFETY] request PX4 LAND: %s",
        reason.c_str());
    }

    ctx_.handover_to_px4_land = true;
    publish_land_command();
  }

  bool setpoint_finite() const
  {
    return std::isfinite(ctx_.sp_x) &&
           std::isfinite(ctx_.sp_y) &&
           std::isfinite(ctx_.sp_z) &&
           std::isfinite(ctx_.sp_yaw);
  }

  void print_state_throttled()
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "[STATE] task=%s done=%d arm=%u nav=%u home=%d home_yaw=%d "
      "pos=(%.2f %.2f %.2f) vel=(%.2f %.2f %.2f) "
      "sp=(%.2f %.2f %.2f) yaw=%.2f sp_yaw=%.2f "
      "takeoff_z=%.2f land=%d",
      sched_.current_name().c_str(),
      sched_.done(),
      static_cast<unsigned>(ctx_.vehicle_status.arming_state),
      static_cast<unsigned>(ctx_.vehicle_status.nav_state),
      ctx_.home_inited,
      ctx_.home_yaw_inited,
      ctx_.local_pos.x,
      ctx_.local_pos.y,
      ctx_.local_pos.z,
      ctx_.local_pos.vx,
      ctx_.local_pos.vy,
      ctx_.local_pos.vz,
      ctx_.sp_x,
      ctx_.sp_y,
      ctx_.sp_z,
      ctx_.yaw,
      ctx_.sp_yaw,
      ctx_.takeoff_z,
      ctx_.land_detected.landed);
  }

  void on_timer()
  {
    const auto t = now();
    double dt = (t - last_time_).seconds();
    last_time_ = t;

    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2) {
      dt = 0.05;
    }

    // 已经请求 Land 时，持续发送 PX4 Land 命令
    if (ctx_.handover_to_px4_land) {
      publish_land_command();
      publish_vision_gate();

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[LAND] handover_to_px4_land=true, keep publishing NAV_LAND");

      return;
    }

    // D435 看到目标后，YawSpinTask 进入 STABILIZING。
    // 这里等待稳定采样时间结束后，置 vision_done=true，让 YawSpin 继续转。
    if (front_stabilizing_) {
      const double hold_s = (t - front_stabilize_start_).seconds();

      if (hold_s >= front_stabilize_s_) {
        ctx_.vision_done = true;
        front_stabilizing_ = false;

        RCLCPP_WARN(
          get_logger(),
          "[VISION_FRONT] stabilize done %.2fs -> resume yaw scan",
          hold_s);
      }
    }

    // home / yaw 没准备好之前，只让 Scheduler 自己停在 WaitHome，不发布 Offboard setpoint
    if (!ctx_.home_inited || !ctx_.home_yaw_inited) {
      sched_.tick(ctx_, dt);
      publish_vision_gate();
      print_state_throttled();
      return;
    }

    // ============================================================
    // 关键修改：
    // 先让当前任务 tick，更新 ctx.sp_x / sp_y / sp_z / sp_yaw；
    // 再发布 offboard_control_mode 和 trajectory_setpoint。
    // 这样 TakeoffTask 写出的 z=-1.0 不会晚一拍或被旧值覆盖。
    // ============================================================
    sched_.tick(ctx_, dt);

    publish_vision_gate();

    print_state_throttled();

    if (sched_.done()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[MISSION] scheduler done");
      return;
    }

    if (!setpoint_finite()) {
      request_land("invalid setpoint before publish");
      return;
    }

    // 起飞阶段如果 sp_z 没有变成负值，直接报警，方便判断空转原因
    if (sched_.current_name() == "TAKEOFF" && ctx_.sp_z > ctx_.home_z - 0.20f) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "[TAKEOFF] abnormal sp_z=%.2f home_z=%.2f takeoff_z=%.2f. "
        "PX4 NED takeoff should use negative z.",
        ctx_.sp_z,
        ctx_.home_z,
        ctx_.takeoff_z);
    }

    px4_->publish_offboard_control_mode(ctx_);
    px4_->publish_setpoint_from_ctx(ctx_);
  }

private:
  Context ctx_;
  std::unique_ptr<Px4Iface> px4_;
  Scheduler sched_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  // ===== parameters =====
  double takeoff_height_m_{1.0};
  double arrival_error_max_{0.25};
  double hover_s_{2.0};
  double yaw_rate_{0.25};

  double front_stabilize_s_{0.5};

  double max_target_xy_m_{4.0};
  double min_target_z_m_{-2.0};
  double max_target_z_m_{0.5};

  // ===== runtime state =====
  bool front_stabilizing_{false};
  rclcpp::Time front_stabilize_start_;

  // ===== publishers =====
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_front_enable_pub_;

  // ===== subscriptions =====
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_att_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_sub_;
  rclcpp::Subscription<custom_vision_msgs::msg::PositionTargetArray>::SharedPtr targets_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardTestNode>());
  rclcpp::shutdown();
  return 0;
}