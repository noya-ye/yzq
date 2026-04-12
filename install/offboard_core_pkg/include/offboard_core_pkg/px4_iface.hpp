#pragma once
#include <limits>
#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

#include "offboard_core_pkg/context.hpp"

class Px4Iface {
public:
  Px4Iface(rclcpp::Node& node,
           rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm,
           rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr tsp,
           rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vcmd)
  : node_(node), ocm_(ocm), tsp_(tsp), vcmd_(vcmd) {}

  uint64_t now_us() const {
    return static_cast<uint64_t>(node_.get_clock()->now().nanoseconds() / 1000ULL);
  }

  void publish_offboard_control_mode(const Context& ctx) {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = now_us();

    // 起飞速度段用 velocity，否则 position
    msg.position = !ctx.use_vel_ctrl;
    msg.velocity =  ctx.use_vel_ctrl;

    msg.acceleration = false;
    msg.attitude     = false;
    msg.body_rate    = false;

    ocm_->publish(msg);
  }

  void publish_setpoint_from_ctx(const Context& ctx) {
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = now_us();

    const float NaN = std::numeric_limits<float>::quiet_NaN();

    // 先把没用字段全置 NaN，避免 PX4 误约束
    sp.position[0] = NaN; sp.position[1] = NaN; sp.position[2] = NaN;
    sp.velocity[0] = NaN; sp.velocity[1] = NaN; sp.velocity[2] = NaN;
    sp.acceleration[0] = NaN; sp.acceleration[1] = NaN; sp.acceleration[2] = NaN;
    sp.jerk[0] = NaN; sp.jerk[1] = NaN; sp.jerk[2] = NaN;

    sp.yaw      = ctx.sp_yaw;
    sp.yawspeed = NaN;

    if (!ctx.use_vel_ctrl) {
      // position control
      sp.position[0] = ctx.sp_x;
      sp.position[1] = ctx.sp_y;
      sp.position[2] = ctx.sp_z;
    } else {
      // velocity control
      sp.velocity[0] = ctx.sp_vx;
      sp.velocity[1] = ctx.sp_vy;
      sp.velocity[2] = ctx.sp_vz;
    }

    tsp_->publish(sp);
  }

  void send_vehicle_command(uint16_t command,
                            float param1=0, float param2=0, float param3=0,
                            float param4=0, float param5=0, float param6=0, float param7=0) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = now_us();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;

    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vcmd_->publish(msg);
  }

  void set_offboard_mode() {
    send_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0f,  // custom mode enabled
      6.0f   // PX4 main mode: OFFBOARD
    );
  }

  void arm() {
    send_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0f
    );
  }

  void disarm() {
    send_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      0.0f
    );
  }

  // =========================
  // 新增：切换到 PX4 自带降落模式
  // =========================
  void land() {
    send_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND
    );
  }

private:
  rclcpp::Node& node_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr tsp_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vcmd_;
};