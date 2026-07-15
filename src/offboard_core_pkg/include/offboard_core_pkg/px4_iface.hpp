#pragma once

#include <cstdint>
#include <limits>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

#include "offboard_core_pkg/context.hpp"

class Px4Iface
{
public:
  Px4Iface(
    rclcpp::Node& node,
    rclcpp::Publisher<
      px4_msgs::msg::OffboardControlMode>::SharedPtr ocm,
    rclcpp::Publisher<
      px4_msgs::msg::TrajectorySetpoint>::SharedPtr tsp,
    rclcpp::Publisher<
      px4_msgs::msg::VehicleCommand>::SharedPtr vcmd)
  : node_(node),
    ocm_(ocm),
    tsp_(tsp),
    vcmd_(vcmd)
  {
  }

  uint64_t now_us() const
  {
    return static_cast<uint64_t>(
      node_.get_clock()->now().nanoseconds() / 1000ULL);
  }

  // ============================================================
  // 发布 PX4 Offboard 控制模式
  //
  // use_vel_ctrl = false:
  //   PX4 使用位置控制器。
  //
  //   use_trajectory_ff = false:
  //     只发送 position。
  //
  //   use_trajectory_ff = true:
  //     发送 position + velocity + acceleration。
  //     position 是主控制目标；
  //     velocity、acceleration 是前馈。
  //
  // use_vel_ctrl = true:
  //   PX4 使用纯速度控制，只发送 velocity。
  // ============================================================
  void publish_offboard_control_mode(const Context& ctx)
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = now_us();

    if (ctx.use_vel_ctrl) {
      // 纯速度控制
      msg.position = false;
      msg.velocity = true;
    } else {
      // 普通位置控制或带前馈的轨迹控制
      msg.position = true;
      msg.velocity = false;
    }

    // 即使使用 acceleration 前馈，
    // 这里也保持 false，因为主控制层仍然是 position。
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    ocm_->publish(msg);
  }

  // ============================================================
  // 发布 PX4 TrajectorySetpoint
  // ============================================================
  void publish_setpoint_from_ctx(const Context& ctx)
  {
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = now_us();

    const float NaN =
      std::numeric_limits<float>::quiet_NaN();

    // 默认所有未使用的字段均为 NaN。
    // NaN 表示不提供该控制量。
    sp.position[0] = NaN;
    sp.position[1] = NaN;
    sp.position[2] = NaN;

    sp.velocity[0] = NaN;
    sp.velocity[1] = NaN;
    sp.velocity[2] = NaN;

    sp.acceleration[0] = NaN;
    sp.acceleration[1] = NaN;
    sp.acceleration[2] = NaN;

    sp.jerk[0] = NaN;
    sp.jerk[1] = NaN;
    sp.jerk[2] = NaN;

    sp.yaw = ctx.sp_yaw;
    sp.yawspeed = NaN;

    if (ctx.use_vel_ctrl) {
      // ========================================================
      // 模式1：纯速度控制
      // ========================================================
      sp.velocity[0] = ctx.sp_vx;
      sp.velocity[1] = ctx.sp_vy;
      sp.velocity[2] = ctx.sp_vz;
    } else {
      // ========================================================
      // 模式2：位置控制
      // ========================================================
      sp.position[0] = ctx.sp_x;
      sp.position[1] = ctx.sp_y;
      sp.position[2] = ctx.sp_z;

      if (ctx.use_trajectory_ff) {
        // ======================================================
        // 模式3：位置控制 + EGO 轨迹前馈
        //
        // position     主控制目标
        // velocity     速度前馈
        // acceleration 加速度前馈
        // ======================================================
        sp.velocity[0] = ctx.sp_vx;
        sp.velocity[1] = ctx.sp_vy;
        sp.velocity[2] = ctx.sp_vz;

        sp.acceleration[0] = ctx.sp_ax;
        sp.acceleration[1] = ctx.sp_ay;
        sp.acceleration[2] = ctx.sp_az;
      }
    }

    tsp_->publish(sp);
  }

  // ============================================================
  // 通用 VehicleCommand
  // ============================================================
  void send_vehicle_command(
    uint16_t command,
    float param1 = 0.0f,
    float param2 = 0.0f,
    float param3 = 0.0f,
    float param4 = 0.0f,
    float param5 = 0.0f,
    float param6 = 0.0f,
    float param7 = 0.0f)
  {
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

  // ============================================================
  // 切换到 Offboard
  // ============================================================
  void set_offboard_mode()
  {
    send_vehicle_command(
      px4_msgs::msg::VehicleCommand::
        VEHICLE_CMD_DO_SET_MODE,
      1.0f,  // custom mode enabled
      6.0f   // PX4 main mode: OFFBOARD
    );
  }

  // ============================================================
  // 解锁
  // ============================================================
  void arm()
  {
    send_vehicle_command(
      px4_msgs::msg::VehicleCommand::
        VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0f);
  }

  // ============================================================
  // 上锁
  // ============================================================
  void disarm()
  {
    send_vehicle_command(
      px4_msgs::msg::VehicleCommand::
        VEHICLE_CMD_COMPONENT_ARM_DISARM,
      0.0f);
  }

  // ============================================================
  // 切换到 PX4 自带降落模式
  // ============================================================
  void land()
  {
    send_vehicle_command(
      px4_msgs::msg::VehicleCommand::
        VEHICLE_CMD_NAV_LAND);
  }

private:
  rclcpp::Node& node_;

  rclcpp::Publisher<
    px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_;

  rclcpp::Publisher<
    px4_msgs::msg::TrajectorySetpoint>::SharedPtr tsp_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleCommand>::SharedPtr vcmd_;
};