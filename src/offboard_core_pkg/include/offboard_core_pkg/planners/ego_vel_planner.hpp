#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg
{

/*
 * EGO 轨迹到 PX4 TrajectorySetpoint 的转换器
 *
 * 控制方式：
 *
 *   PX4 OffboardControlMode:
 *     position = true
 *
 *   PX4 TrajectorySetpoint:
 *     position     = 主控制目标
 *     velocity     = EGO 速度前馈
 *     acceleration = EGO 加速度前馈
 *
 * 坐标转换方式：
 *
 *   不直接将 EGO 的绝对坐标当作 PX4 local 坐标。
 *
 *   使用：
 *
 *     EGO误差 = EGO规划位置 - FAST-LIO实际位置
 *
 *   然后把该误差旋转、换轴后叠加到 PX4 当前坐标。
 *
 * 这样不要求：
 *
 *   FAST-LIO 世界系原点 == PX4 local NED 原点
 */
class EgoVelPlanner
{
public:
  struct Config
  {
    // 输入超时保护
    double cmd_timeout_s{0.50};
    double odom_timeout_s{0.50};

    // 单周期位置设定值最大变化
    // <= 0 表示关闭该限制
    double max_step_m{0.10};

    // EGO位置误差到PX4位置修正的比例
    //
    // 1.0：
    //   完整跟随 EGO 的相对位置误差
    //
    // 小于1.0：
    //   更保守，但响应更慢
    double kp_xy{1.00};
    double kp_z{1.00};

    // 相对于PX4当前位置，单次允许发送的最大位置偏移
    double max_cmd_xy_m{0.30};
    double max_cmd_z_m{0.15};

    // 是否发送EGO速度前馈
    bool use_velocity_ff{true};

    // 是否发送EGO加速度前馈
    bool use_acceleration_ff{false};

    // 前馈缩放
    double vel_ff_scale{0.50};
    double acc_ff_scale{0.00};

    /*
     * 可选的速度误差修正。
     *
     * 正常使用 position + velocity feedforward 时，
     * PX4自身已经会根据位置误差产生速度修正，
     * 因此建议保持为0。
     */
    double kp_vel_xy{0.00};
    double kp_vel_z{0.00};

    // 速度前馈限幅
    double max_vel_xy_mps{0.50};
    double max_vel_z_mps{0.30};

    // 加速度前馈限幅
    double max_acc_xy_mps2{0.60};
    double max_acc_z_mps2{0.40};

    // EGO命令与FAST-LIO里程计相差过大时悬停
    double err_xy_hold_m{2.0};
    double err_z_hold_m{1.0};

    // EGO XY 到 PX4 local XY 的映射参数
    double x_sign{1.0};
    double y_sign{1.0};
    bool swap_xy{true};

    // EGO平面相对于PX4平面的旋转
    double yaw_align_rad{0.0};

    // 防止高度指令接近或低于起飞前地面高度
    double min_takeoff_height_m{0.20};

    // 是否使用EGO输出yaw
    bool use_ego_yaw{false};
  };

  enum class Result
  {
    OK,
    HOLD_NO_PX4_POSITION,
    HOLD_STALE_EGO_CMD,
    HOLD_STALE_EGO_ODOM,
    HOLD_EGO_ERROR_TOO_LARGE
  };

  struct Debug
  {
    double cmd_age_s{999.0};
    double odom_age_s{999.0};

    // EGO世界系下：
    // position_cmd - FAST-LIO odom
    double dx_ego{0.0};
    double dy_ego{0.0};
    double dz_ego{0.0};
    double err_xy{0.0};

    // 映射到PX4 local后的XY误差
    double ex_map{0.0};
    double ey_map{0.0};

    // 最终写入Context的前馈
    double vx_cmd{0.0};
    double vy_cmd{0.0};
    double vz_cmd{0.0};

    double ax_cmd{0.0};
    double ay_cmd{0.0};
    double az_cmd{0.0};

    std::string reason;
  };

  EgoVelPlanner()
  : cfg_()
  {
  }

  explicit EgoVelPlanner(const Config& cfg)
  : cfg_(cfg)
  {
  }

  void setConfig(const Config& cfg)
  {
    cfg_ = cfg;
  }

  const Config& config() const
  {
    return cfg_;
  }

  void reset(Context& ctx)
  {
    hold_yaw_ =
      ctx.home_yaw_inited
        ? ctx.home_yaw
        : ctx.yaw;

    last_sp_x_ = ctx.cx();
    last_sp_y_ = ctx.cy();
    last_sp_z_ = ctx.cz();

    // EGO使用位置主控制
    ctx.use_vel_ctrl = false;

    // 是否启用轨迹前馈由配置决定
    ctx.use_trajectory_ff =
      cfg_.use_velocity_ff ||
      cfg_.use_acceleration_ff;

    setUnusedFeedforward(ctx);

    inited_ = true;
  }

  Result plan(Context& ctx, Debug* debug = nullptr)
  {
    Debug local_debug;
    Debug& dbg = debug ? *debug : local_debug;

    // ============================================================
    // 0. 固定使用PX4位置控制
    // ============================================================
    ctx.use_vel_ctrl = false;

    if (!inited_) {
      reset(ctx);
    }

    if (!ctx.pos_valid()) {
      holdCurrent(ctx);
      dbg.reason = "waiting PX4 local position valid";
      return Result::HOLD_NO_PX4_POSITION;
    }

    const uint64_t now_us = nowUs();

    dbg.cmd_age_s =
      ctx.ego_cmd_valid
        ? calculateAgeSeconds(now_us, ctx.ego_cmd_stamp_us)
        : 999.0;

    dbg.odom_age_s =
      ctx.ego_odom_valid
        ? calculateAgeSeconds(now_us, ctx.ego_odom_stamp_us)
        : 999.0;

    if (!ctx.ego_cmd_valid ||
        dbg.cmd_age_s > cfg_.cmd_timeout_s) {
      holdCurrent(ctx);
      dbg.reason = "stale /position_cmd";
      return Result::HOLD_STALE_EGO_CMD;
    }

    if (!ctx.ego_odom_valid ||
        dbg.odom_age_s > cfg_.odom_timeout_s) {
      holdCurrent(ctx);
      dbg.reason = "stale /fastlio2/lio_odom";
      return Result::HOLD_STALE_EGO_ODOM;
    }

    // 有效轨迹恢复前馈模式
    ctx.use_trajectory_ff =
      cfg_.use_velocity_ff ||
      cfg_.use_acceleration_ff;

    // ============================================================
    // 1. 计算EGO世界系下的跟踪误差
    // ============================================================
    dbg.dx_ego =
      ctx.ego_x - ctx.ego_odom_x;

    dbg.dy_ego =
      ctx.ego_y - ctx.ego_odom_y;

    dbg.dz_ego =
      ctx.ego_z - ctx.ego_odom_z;

    dbg.err_xy = std::hypot(
      dbg.dx_ego,
      dbg.dy_ego);

    if (dbg.err_xy > cfg_.err_xy_hold_m ||
        std::fabs(dbg.dz_ego) > cfg_.err_z_hold_m) {
      holdCurrent(ctx);
      dbg.reason = "EGO error too large";
      return Result::HOLD_EGO_ERROR_TOO_LARGE;
    }

    // ============================================================
    // 2. 将EGO位置误差映射到PX4 local XY
    // ============================================================
    mapXY(
      dbg.dx_ego,
      dbg.dy_ego,
      dbg.ex_map,
      dbg.ey_map);

    // ============================================================
    // 3. 计算PX4位置主目标
    //
    // 不使用EGO绝对坐标，避免FAST-LIO原点与PX4原点不一致。
    //
    // 使用：
    //
    //   PX4目标 = PX4当前位置 + 映射后的EGO相对误差
    // ============================================================
    double cmd_dx =
      cfg_.kp_xy * dbg.ex_map;

    double cmd_dy =
      cfg_.kp_xy * dbg.ey_map;

    limitXY(
      cmd_dx,
      cmd_dy,
      cfg_.max_cmd_xy_m);

    double cmd_dz =
      -cfg_.kp_z * dbg.dz_ego;

    cmd_dz = std::clamp(
      cmd_dz,
      -cfg_.max_cmd_z_m,
      cfg_.max_cmd_z_m);

    float target_x =
      static_cast<float>(ctx.cx() + cmd_dx);

    float target_y =
      static_cast<float>(ctx.cy() + cmd_dy);

    // FAST-LIO z向上为正，PX4 NED z向下为正
    float target_z =
      static_cast<float>(ctx.cz() + cmd_dz);

    // XY整体限步，避免斜向运动时每个轴分别限幅导致总步长变大
    clampStepXY(
      last_sp_x_,
      last_sp_y_,
      target_x,
      target_y,
      static_cast<float>(cfg_.max_step_m));

    target_z = clampStep(
      last_sp_z_,
      target_z,
      static_cast<float>(cfg_.max_step_m));

    // ============================================================
    // 4. 高度安全保护
    // ============================================================
    if (ctx.home_inited) {
      // PX4 NED：
      // z越大表示越低
      const float lowest_allowed_z =
        static_cast<float>(
          ctx.home_z -
          cfg_.min_takeoff_height_m);

      if (target_z > lowest_allowed_z) {
        target_z = lowest_allowed_z;
      }
    }

    // ============================================================
    // 5. EGO速度前馈
    // ============================================================
    if (cfg_.use_velocity_ff) {
      double vx_ff = 0.0;
      double vy_ff = 0.0;

      mapXY(
        ctx.ego_vx,
        ctx.ego_vy,
        vx_ff,
        vy_ff);

      vx_ff *= cfg_.vel_ff_scale;
      vy_ff *= cfg_.vel_ff_scale;

      double vz_ff =
        -ctx.ego_vz *
        cfg_.vel_ff_scale;

      /*
       * 默认kp_vel为0。
       *
       * 如果后续确实需要增加少量外部修正，
       * 才设置非零值。
       */
      dbg.vx_cmd =
        vx_ff +
        cfg_.kp_vel_xy * dbg.ex_map;

      dbg.vy_cmd =
        vy_ff +
        cfg_.kp_vel_xy * dbg.ey_map;

      dbg.vz_cmd =
        vz_ff +
        cfg_.kp_vel_z * (-dbg.dz_ego);

      limitXY(
        dbg.vx_cmd,
        dbg.vy_cmd,
        cfg_.max_vel_xy_mps);

      dbg.vz_cmd = std::clamp(
        dbg.vz_cmd,
        -cfg_.max_vel_z_mps,
        cfg_.max_vel_z_mps);

      ctx.sp_vx =
        static_cast<float>(dbg.vx_cmd);

      ctx.sp_vy =
        static_cast<float>(dbg.vy_cmd);

      ctx.sp_vz =
        static_cast<float>(dbg.vz_cmd);
    } else {
      setVelocityUnused(ctx);

      dbg.vx_cmd = quietNaN();
      dbg.vy_cmd = quietNaN();
      dbg.vz_cmd = quietNaN();
    }

    // ============================================================
    // 6. EGO加速度前馈
    // ============================================================
    if (cfg_.use_acceleration_ff) {
      double ax_ff = 0.0;
      double ay_ff = 0.0;

      mapXY(
        ctx.ego_ax,
        ctx.ego_ay,
        ax_ff,
        ay_ff);

      ax_ff *= cfg_.acc_ff_scale;
      ay_ff *= cfg_.acc_ff_scale;

      double az_ff =
        -ctx.ego_az *
        cfg_.acc_ff_scale;

      dbg.ax_cmd = ax_ff;
      dbg.ay_cmd = ay_ff;
      dbg.az_cmd = az_ff;

      limitXY(
        dbg.ax_cmd,
        dbg.ay_cmd,
        cfg_.max_acc_xy_mps2);

      dbg.az_cmd = std::clamp(
        dbg.az_cmd,
        -cfg_.max_acc_z_mps2,
        cfg_.max_acc_z_mps2);

      ctx.sp_ax =
        static_cast<float>(dbg.ax_cmd);

      ctx.sp_ay =
        static_cast<float>(dbg.ay_cmd);

      ctx.sp_az =
        static_cast<float>(dbg.az_cmd);
    } else {
      setAccelerationUnused(ctx);

      dbg.ax_cmd = quietNaN();
      dbg.ay_cmd = quietNaN();
      dbg.az_cmd = quietNaN();
    }

    // ============================================================
    // 7. 写入PX4位置主目标
    // ============================================================
    ctx.sp_x = target_x;
    ctx.sp_y = target_y;
    ctx.sp_z = target_z;

    if (cfg_.use_ego_yaw &&
        std::isfinite(ctx.ego_yaw)) {
      ctx.sp_yaw =
        static_cast<float>(ctx.ego_yaw);
    } else {
      ctx.sp_yaw = hold_yaw_;
    }

    last_sp_x_ = ctx.sp_x;
    last_sp_y_ = ctx.sp_y;
    last_sp_z_ = ctx.sp_z;

    dbg.reason = "OK";
    return Result::OK;
  }

  static const char* resultName(Result result)
  {
    switch (result) {
      case Result::OK:
        return "OK";

      case Result::HOLD_NO_PX4_POSITION:
        return "HOLD_NO_PX4_POSITION";

      case Result::HOLD_STALE_EGO_CMD:
        return "HOLD_STALE_EGO_CMD";

      case Result::HOLD_STALE_EGO_ODOM:
        return "HOLD_STALE_EGO_ODOM";

      case Result::HOLD_EGO_ERROR_TOO_LARGE:
        return "HOLD_EGO_ERROR_TOO_LARGE";

      default:
        return "UNKNOWN";
    }
  }

private:
  // ============================================================
  // 异常时退回普通PX4位置悬停
  //
  // 关闭轨迹前馈，防止旧速度/加速度继续生效。
  // ============================================================
  void holdCurrent(Context& ctx)
  {
    ctx.use_vel_ctrl = false;
    ctx.use_trajectory_ff = false;

    ctx.sp_x = ctx.cx();
    ctx.sp_y = ctx.cy();
    ctx.sp_z = ctx.cz();
    ctx.sp_yaw = hold_yaw_;

    setUnusedFeedforward(ctx);

    last_sp_x_ = ctx.sp_x;
    last_sp_y_ = ctx.sp_y;
    last_sp_z_ = ctx.sp_z;
  }

  void setUnusedFeedforward(Context& ctx)
  {
    setVelocityUnused(ctx);
    setAccelerationUnused(ctx);
  }

  static void setVelocityUnused(Context& ctx)
  {
    const float nan =
      std::numeric_limits<float>::quiet_NaN();

    ctx.sp_vx = nan;
    ctx.sp_vy = nan;
    ctx.sp_vz = nan;
  }

  static void setAccelerationUnused(Context& ctx)
  {
    const float nan =
      std::numeric_limits<float>::quiet_NaN();

    ctx.sp_ax = nan;
    ctx.sp_ay = nan;
    ctx.sp_az = nan;
  }

  // ============================================================
  // EGO XY映射到PX4 local XY
  // ============================================================
  void mapXY(
    double x_in,
    double y_in,
    double& x_out,
    double& y_out) const
  {
    double x = x_in;
    double y = y_in;

    if (cfg_.swap_xy) {
      std::swap(x, y);
    }

    const double c =
      std::cos(cfg_.yaw_align_rad);

    const double s =
      std::sin(cfg_.yaw_align_rad);

    const double rotated_x =
      c * x - s * y;

    const double rotated_y =
      s * x + c * y;

    x_out =
      cfg_.x_sign * rotated_x;

    y_out =
      cfg_.y_sign * rotated_y;
  }

  // ============================================================
  // 限制XY向量模长
  // ============================================================
  static void limitXY(
    double& x,
    double& y,
    double max_norm)
  {
    if (max_norm <= 0.0) {
      return;
    }

    const double norm =
      std::hypot(x, y);

    if (norm <= max_norm ||
        norm <= 1e-9) {
      return;
    }

    const double scale =
      max_norm / norm;

    x *= scale;
    y *= scale;
  }

  // ============================================================
  // XY整体限步
  // ============================================================
  static void clampStepXY(
    float current_x,
    float current_y,
    float& target_x,
    float& target_y,
    float max_step)
  {
    if (max_step <= 0.0f) {
      return;
    }

    const float dx =
      target_x - current_x;

    const float dy =
      target_y - current_y;

    const float distance =
      std::hypot(dx, dy);

    if (distance <= max_step ||
        distance <= 1e-6f) {
      return;
    }

    const float scale =
      max_step / distance;

    target_x =
      current_x + dx * scale;

    target_y =
      current_y + dy * scale;
  }

  static float clampStep(
    float current,
    float target,
    float max_step)
  {
    if (max_step <= 0.0f) {
      return target;
    }

    const float delta =
      target - current;

    return current +
      std::clamp(
        delta,
        -max_step,
        max_step);
  }

  static double calculateAgeSeconds(
    uint64_t now_us,
    uint64_t stamp_us)
  {
    if (stamp_us == 0 ||
        now_us < stamp_us) {
      return 999.0;
    }

    return static_cast<double>(
      now_us - stamp_us) / 1e6;
  }

  static double quietNaN()
  {
    return
      std::numeric_limits<double>::quiet_NaN();
  }

  static uint64_t nowUs()
  {
    return static_cast<uint64_t>(
      rclcpp::Clock().now().nanoseconds() /
      1000ULL);
  }

private:
  Config cfg_;

  bool inited_{false};

  float hold_yaw_{0.0f};

  float last_sp_x_{0.0f};
  float last_sp_y_{0.0f};
  float last_sp_z_{0.0f};
};

}  // namespace offboard_core_pkg