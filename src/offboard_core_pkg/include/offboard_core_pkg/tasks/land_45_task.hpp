#pragma once

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/itask.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace offboard_core_pkg
{

/*
 * 45°斜向降落任务。
 *
 * 轨迹分为三段：
 *   1. 保持当前高度，移动到起飞点外侧的斜降起点；
 *   2. 水平距离与下降高度按 1:1 同步减小，形成约 45°下降轨迹；
 *   3. 到达低空交接高度后结束，由 Px4LandModeTask 完成最后触地。
 *
 * PX4 local position 使用 NED：
 *   - z 越小表示越高；
 *   - home_z 为地面高度；
 *   - 当前离地高度约为 home_z - current_z。
 */
class Land45Task : public ITask
{
public:
  struct Config
  {
    // 斜降起点相对 home 的水平方向，会在任务内部归一化。
    // 对当前 A9B1 起飞点位于地图右下角的布局，可优先使用 (0, 1)。
    double approach_dir_x{0.0};
    double approach_dir_y{1.0};

    // 进入斜降前，移动到起点时的最大位置指令步长。
    double approach_step_m{0.04};

    // 斜降阶段每次 tick 沿三维轨迹前进的最大距离。
    double descend_step_m{0.025};

    double approach_xy_tol_m{0.08};
    double approach_z_tol_m{0.08};

    // 距离地面达到该高度时结束任务，交给 Px4LandModeTask。
    double handover_height_m{0.18};

    // 防止初始高度异常导致斜降起点过远。
    double max_horizontal_offset_m{1.50};

    // 到达斜降起点后稳定一小段时间再下降。
    double approach_hold_s{0.40};

    // LED 闪烁半周期。
    double led_toggle_s{0.25};

    double timeout_s{30.0};
  };

  Land45Task(
    rclcpp::Logger logger,
    const Config& cfg,
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pub)
  : logger_(logger), cfg_(cfg), led_pub_(std::move(led_pub))
  {
  }

  std::string name() const override
  {
    return "LAND_45";
  }

  void onEnter(Context& ctx) override
  {
    phase_ = Phase::MOVE_TO_APPROACH;
    elapsed_s_ = 0.0;
    hold_elapsed_s_ = 0.0;
    led_elapsed_s_ = 0.0;
    led_state_ = false;

    if (!ctx.home_inited || !ctx.pos_valid()) {
      fail("home or local position invalid");
      return;
    }

    double dir_norm = std::hypot(cfg_.approach_dir_x, cfg_.approach_dir_y);
    if (!std::isfinite(dir_norm) || dir_norm < 1e-6) {
      fail("approach direction is invalid");
      return;
    }

    dir_x_ = cfg_.approach_dir_x / dir_norm;
    dir_y_ = cfg_.approach_dir_y / dir_norm;

    home_x_ = ctx.home_x;
    home_y_ = ctx.home_y;
    home_z_ = ctx.home_z;
    cruise_z_ = ctx.cz();

    const double height = home_z_ - cruise_z_;
    if (!std::isfinite(height) || height <= cfg_.handover_height_m) {
      fail("initial height is too low");
      return;
    }

    /*
     * 45°轨迹要求：
     *   水平总距离 = 垂直下降高度。
     *
     * 最后 handover_height_m 由 PX4 Land 完成，因此本段垂直下降量为：
     *   height - handover_height_m
     */
    horizontal_offset_m_ = std::clamp(
      height - cfg_.handover_height_m,
      0.0,
      cfg_.max_horizontal_offset_m);

    approach_x_ = home_x_ + dir_x_ * horizontal_offset_m_;
    approach_y_ = home_y_ + dir_y_ * horizontal_offset_m_;
    approach_z_ = cruise_z_;

    descend_end_x_ = home_x_;
    descend_end_y_ = home_y_;
    descend_end_z_ = home_z_ - cfg_.handover_height_m;

    cmd_x_ = ctx.cx();
    cmd_y_ = ctx.cy();
    cmd_z_ = ctx.cz();

    yaw_hold_ = std::isfinite(ctx.sp_yaw) ? ctx.sp_yaw : ctx.yaw;
    publishSetpoint(ctx);
    publishLed(false);

    RCLCPP_WARN(
      logger_,
      "[LAND_45] enter: home=(%.2f %.2f %.2f) approach=(%.2f %.2f %.2f) "
      "end=(%.2f %.2f %.2f) horizontal=%.2f",
      home_x_, home_y_, home_z_,
      approach_x_, approach_y_, approach_z_,
      descend_end_x_, descend_end_y_, descend_end_z_,
      horizontal_offset_m_);
  }

  Status tick(Context& ctx, double dt_s) override
  {
    if (phase_ == Phase::FAILED) {
      publishSetpoint(ctx);
      return Status::FAILURE;
    }

    if (phase_ == Phase::FINISHED) {
      publishSetpoint(ctx);
      return Status::SUCCESS;
    }

    const double dt = std::max(0.0, dt_s);
    elapsed_s_ += dt;

    if (elapsed_s_ > cfg_.timeout_s) {
      fail("timeout");
      publishSetpoint(ctx);
      return Status::FAILURE;
    }

    if (!ctx.pos_valid()) {
      publishSetpoint(ctx);
      updateLed(dt);
      return Status::RUNNING;
    }

    if (phase_ == Phase::MOVE_TO_APPROACH) {
      moveToward(approach_x_, approach_y_, approach_z_, cfg_.approach_step_m);
      publishSetpoint(ctx);

      if (arrived(
          ctx,
          approach_x_,
          approach_y_,
          approach_z_,
          cfg_.approach_xy_tol_m,
          cfg_.approach_z_tol_m)) {
        cmd_x_ = approach_x_;
        cmd_y_ = approach_y_;
        cmd_z_ = approach_z_;
        hold_elapsed_s_ = 0.0;
        phase_ = Phase::HOLD_APPROACH;

        RCLCPP_WARN(logger_, "[LAND_45] approach point reached");
      }

      return Status::RUNNING;
    }

    if (phase_ == Phase::HOLD_APPROACH) {
      cmd_x_ = approach_x_;
      cmd_y_ = approach_y_;
      cmd_z_ = approach_z_;
      publishSetpoint(ctx);

      hold_elapsed_s_ += dt;
      if (hold_elapsed_s_ >= cfg_.approach_hold_s) {
        phase_ = Phase::DESCENDING;
        led_elapsed_s_ = 0.0;
        led_state_ = true;
        publishLed(true);

        RCLCPP_WARN(logger_, "[LAND_45] diagonal descent started, LED flashing");
      }

      return Status::RUNNING;
    }

    if (phase_ == Phase::DESCENDING) {
      /*
       * 对完整三维线段做等比例推进。
       * 因为水平总距离等于垂直下降量，所以轨迹俯角约为 45°。
       */
      moveToward(
        descend_end_x_,
        descend_end_y_,
        descend_end_z_,
        cfg_.descend_step_m);

      publishSetpoint(ctx);
      updateLed(dt);

      if (arrived(
          ctx,
          descend_end_x_,
          descend_end_y_,
          descend_end_z_,
          cfg_.approach_xy_tol_m,
          cfg_.approach_z_tol_m)) {
        cmd_x_ = descend_end_x_;
        cmd_y_ = descend_end_y_;
        cmd_z_ = descend_end_z_;
        publishSetpoint(ctx);
        publishLed(false);
        phase_ = Phase::FINISHED;

        RCLCPP_WARN(
          logger_,
          "[LAND_45] handover reached: pos=(%.2f %.2f %.2f), "
          "next task should be PX4_LAND_MODE",
          ctx.cx(), ctx.cy(), ctx.cz());

        return Status::SUCCESS;
      }

      return Status::RUNNING;
    }

    return Status::RUNNING;
  }

  void onExit(Context& ctx) override
  {
    publishLed(false);
    publishSetpoint(ctx);
  }

private:
  enum class Phase
  {
    MOVE_TO_APPROACH,
    HOLD_APPROACH,
    DESCENDING,
    FINISHED,
    FAILED
  };

  void moveToward(double target_x, double target_y, double target_z, double max_step)
  {
    const double dx = target_x - cmd_x_;
    const double dy = target_y - cmd_y_;
    const double dz = target_z - cmd_z_;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 1e-9 || max_step <= 0.0 || distance <= max_step) {
      cmd_x_ = target_x;
      cmd_y_ = target_y;
      cmd_z_ = target_z;
      return;
    }

    const double scale = max_step / distance;
    cmd_x_ += dx * scale;
    cmd_y_ += dy * scale;
    cmd_z_ += dz * scale;
  }

  static bool arrived(
    const Context& ctx,
    double target_x,
    double target_y,
    double target_z,
    double xy_tol,
    double z_tol)
  {
    const double dx = target_x - ctx.cx();
    const double dy = target_y - ctx.cy();
    const double dz = target_z - ctx.cz();

    return std::hypot(dx, dy) <= xy_tol &&
      std::fabs(dz) <= z_tol;
  }

  void publishSetpoint(Context& ctx) const
  {
    ctx.use_vel_ctrl = false;
    ctx.sp_x = static_cast<float>(cmd_x_);
    ctx.sp_y = static_cast<float>(cmd_y_);
    ctx.sp_z = static_cast<float>(cmd_z_);
    ctx.sp_yaw = static_cast<float>(yaw_hold_);
  }

  void updateLed(double dt_s)
  {
    if (phase_ != Phase::DESCENDING) {
      return;
    }

    led_elapsed_s_ += dt_s;
    if (led_elapsed_s_ < cfg_.led_toggle_s) {
      return;
    }

    led_elapsed_s_ = 0.0;
    led_state_ = !led_state_;
    publishLed(led_state_);
  }

  void publishLed(bool enabled)
  {
    if (!led_pub_) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = enabled;
    led_pub_->publish(msg);
  }

  void fail(const std::string& reason)
  {
    phase_ = Phase::FAILED;
    publishLed(false);
    RCLCPP_ERROR(logger_, "[LAND_45] failed: %s", reason.c_str());
  }

  rclcpp::Logger logger_;
  Config cfg_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pub_;

  Phase phase_{Phase::MOVE_TO_APPROACH};

  double elapsed_s_{0.0};
  double hold_elapsed_s_{0.0};
  double led_elapsed_s_{0.0};
  bool led_state_{false};

  double dir_x_{0.0};
  double dir_y_{1.0};

  double home_x_{0.0};
  double home_y_{0.0};
  double home_z_{0.0};
  double cruise_z_{0.0};

  double approach_x_{0.0};
  double approach_y_{0.0};
  double approach_z_{0.0};

  double descend_end_x_{0.0};
  double descend_end_y_{0.0};
  double descend_end_z_{0.0};

  double horizontal_offset_m_{0.0};

  double cmd_x_{0.0};
  double cmd_y_{0.0};
  double cmd_z_{0.0};
  double yaw_hold_{0.0};
};

}  // namespace offboard_core_pkg
