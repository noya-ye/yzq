// 最小模板

// 你以后任何任务想调用 EGO，都按这个模板：

// #include "offboard_core_pkg/planners/ego_vel_planner.hpp"

// class XxxTask : public ITask
// {
// public:
//   XxxTask(rclcpp::Logger logger, const EgoVelPlanner::Config& ego_cfg)
//   : logger_(logger),
//     ego_planner_(ego_cfg)
//   {}

//   void onEnter(Context& ctx) override
//   {
//     ctx.use_vel_ctrl = true;
//     ego_planner_.reset(ctx);
//   }

//   Status tick(Context& ctx, double dt) override
//   {
//     (void)dt;

//     ctx.use_vel_ctrl = true;

//     auto ret = ego_planner_.plan(ctx);

//     if (ret != EgoVelPlanner::Result::OK) {
//       return Status::RUNNING;
//     }

//     return Status::RUNNING;
//   }

// private:
//   rclcpp::Logger logger_;
//   EgoVelPlanner ego_planner_;
// };

// 一句话总结：

// 其他任务想用 EGO，就在任务内部持有 EgoVelPlanner，
// onEnter 里 reset(ctx)，
// tick 里 plan(ctx)，
// plan 会自动把 EGO 结果转换成 PX4 setpoint 写入 ctx。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg
{

class EgoVelPlanner
{
public:
  struct Config
  {
    double cmd_timeout_s{0.50};
    double odom_timeout_s{0.50};

    // 位置目标单周期最大变化，防止 setpoint 跳变
    double max_step_m{0.04};

    // 位置误差反馈增益
    double kp_xy{0.25};
    double kp_z{0.35};

    // 位置误差反馈造成的最大位置偏移
    double max_cmd_xy_m{0.20};
    double max_cmd_z_m{0.10};

    // 速度前馈开关
    bool use_velocity_ff{true};

    // 加速度前馈开关
    bool use_acceleration_ff{true};

    // 速度前馈比例，初期不要直接给 1.0，先保守一点
    double vel_ff_scale{0.60};

    // 加速度前馈比例
    double acc_ff_scale{0.40};

    // 误差反馈速度增益
    double kp_vel_xy{0.45};
    double kp_vel_z{0.35};

    // PX4 速度限幅
    double max_vel_xy_mps{0.60};
    double max_vel_z_mps{0.35};

    // PX4 加速度限幅
    double max_acc_xy_mps2{0.80};
    double max_acc_z_mps2{0.50};

    // EGO 误差过大时悬停，防止坐标系错或者发散
    double err_xy_hold_m{2.0};
    double err_z_hold_m{1.0};

    // 坐标映射参数
    double x_sign{-1.0};
    double y_sign{-1.0};
    bool swap_xy{false};

    // 平面旋转角，单位 rad
    double yaw_align_rad{0.0};

    // PX4 local_position 是 NED，z 越小越高
    double min_takeoff_height_m{0.20};

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

    double dx_ego{0.0};
    double dy_ego{0.0};
    double dz_ego{0.0};
    double err_xy{0.0};

    double ex_map{0.0};
    double ey_map{0.0};

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
{}

explicit EgoVelPlanner(const Config& cfg)
: cfg_(cfg)
{}

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
    hold_yaw_ = ctx.home_yaw_inited ? ctx.home_yaw : ctx.yaw;

    last_sp_x_ = ctx.cx();
    last_sp_y_ = ctx.cy();
    last_sp_z_ = ctx.cz();

    clearVelocityAndAcceleration(ctx);
    inited_ = true;
  }

  Result plan(Context& ctx, Debug* debug = nullptr)
  {
    ctx.use_vel_ctrl = true;

    Debug local_debug;
    Debug& dbg = debug ? *debug : local_debug;

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
        ? static_cast<double>(now_us - ctx.ego_cmd_stamp_us) / 1e6
        : 999.0;

    dbg.odom_age_s =
      ctx.ego_odom_valid
        ? static_cast<double>(now_us - ctx.ego_odom_stamp_us) / 1e6
        : 999.0;

    if (!ctx.ego_cmd_valid || dbg.cmd_age_s > cfg_.cmd_timeout_s) {
      holdCurrent(ctx);
      dbg.reason = "stale /position_cmd";
      return Result::HOLD_STALE_EGO_CMD;
    }

    if (!ctx.ego_odom_valid || dbg.odom_age_s > cfg_.odom_timeout_s) {
      holdCurrent(ctx);
      dbg.reason = "stale /Odometry";
      return Result::HOLD_STALE_EGO_ODOM;
    }

    // ============================================================
    // 1. EGO 世界系误差
    // ============================================================
    dbg.dx_ego = ctx.ego_x - ctx.ego_odom_x;
    dbg.dy_ego = ctx.ego_y - ctx.ego_odom_y;
    dbg.dz_ego = ctx.ego_z - ctx.ego_odom_z;

    dbg.err_xy = std::sqrt(
      dbg.dx_ego * dbg.dx_ego + dbg.dy_ego * dbg.dy_ego);

    if (dbg.err_xy > cfg_.err_xy_hold_m ||
        std::fabs(dbg.dz_ego) > cfg_.err_z_hold_m) {
      holdCurrent(ctx);
      dbg.reason = "EGO error too large";
      return Result::HOLD_EGO_ERROR_TOO_LARGE;
    }

    // ============================================================
    // 2. 位置误差映射到 PX4 local XY
    // ============================================================
    mapXY(dbg.dx_ego, dbg.dy_ego, dbg.ex_map, dbg.ey_map);

    // ============================================================
    // 3. EGO 速度前馈映射到 PX4 local XY
    // ============================================================
    double vx_ff = 0.0;
    double vy_ff = 0.0;

    if (cfg_.use_velocity_ff) {
      mapXY(ctx.ego_vx, ctx.ego_vy, vx_ff, vy_ff);
      vx_ff *= cfg_.vel_ff_scale;
      vy_ff *= cfg_.vel_ff_scale;
    }

    // EGO z 一般向上为正，PX4 NED z 向下为正，所以取反
    double vz_ff = 0.0;
    if (cfg_.use_velocity_ff) {
      vz_ff = -ctx.ego_vz * cfg_.vel_ff_scale;
    }

    // ============================================================
    // 4. EGO 加速度前馈映射到 PX4 local XY
    // ============================================================
    double ax_ff = 0.0;
    double ay_ff = 0.0;

    if (cfg_.use_acceleration_ff) {
      mapXY(ctx.ego_ax, ctx.ego_ay, ax_ff, ay_ff);
      ax_ff *= cfg_.acc_ff_scale;
      ay_ff *= cfg_.acc_ff_scale;
    }

    double az_ff = 0.0;
    if (cfg_.use_acceleration_ff) {
      az_ff = -ctx.ego_az * cfg_.acc_ff_scale;
    }

    // ============================================================
    // 5. 位置 setpoint：当前位置 + 小步位置修正
    // ============================================================
    double cmd_dx = dbg.ex_map * cfg_.kp_xy;
    double cmd_dy = dbg.ey_map * cfg_.kp_xy;

    cmd_dx = std::clamp(cmd_dx, -cfg_.max_cmd_xy_m, cfg_.max_cmd_xy_m);
    cmd_dy = std::clamp(cmd_dy, -cfg_.max_cmd_xy_m, cfg_.max_cmd_xy_m);

    float target_x = static_cast<float>(ctx.cx() + cmd_dx);
    float target_y = static_cast<float>(ctx.cy() + cmd_dy);

    double cmd_dz = -dbg.dz_ego * cfg_.kp_z;
    cmd_dz = std::clamp(cmd_dz, -cfg_.max_cmd_z_m, cfg_.max_cmd_z_m);

    float target_z = static_cast<float>(ctx.cz() + cmd_dz);

    target_x = clampStep(last_sp_x_, target_x, static_cast<float>(cfg_.max_step_m));
    target_y = clampStep(last_sp_y_, target_y, static_cast<float>(cfg_.max_step_m));
    target_z = clampStep(last_sp_z_, target_z, static_cast<float>(cfg_.max_step_m));

    // 高度保护：PX4 NED 下，z 越大越低
    if (ctx.home_inited) {
      const float lowest_allowed_z =
        static_cast<float>(ctx.home_z - cfg_.min_takeoff_height_m);

      if (target_z > lowest_allowed_z) {
        target_z = lowest_allowed_z;
      }
    }

    // ============================================================
    // 6. 速度 setpoint：EGO 速度前馈 + 误差反馈速度
    // ============================================================
    dbg.vx_cmd = vx_ff + cfg_.kp_vel_xy * dbg.ex_map;
    dbg.vy_cmd = vy_ff + cfg_.kp_vel_xy * dbg.ey_map;
    dbg.vz_cmd = vz_ff + cfg_.kp_vel_z * (-dbg.dz_ego);

    limitXY(dbg.vx_cmd, dbg.vy_cmd, cfg_.max_vel_xy_mps);
    dbg.vz_cmd = std::clamp(
      dbg.vz_cmd, -cfg_.max_vel_z_mps, cfg_.max_vel_z_mps);

    // ============================================================
    // 7. 加速度 setpoint：EGO 加速度前馈
    // ============================================================
    dbg.ax_cmd = ax_ff;
    dbg.ay_cmd = ay_ff;
    dbg.az_cmd = az_ff;

    limitXY(dbg.ax_cmd, dbg.ay_cmd, cfg_.max_acc_xy_mps2);
    dbg.az_cmd = std::clamp(
      dbg.az_cmd, -cfg_.max_acc_z_mps2, cfg_.max_acc_z_mps2);

    // ============================================================
    // 8. 写入 Context，交给 PX4 publisher 发 TrajectorySetpoint
    // ============================================================
    ctx.sp_x = target_x;
    ctx.sp_y = target_y;
    ctx.sp_z = target_z;

    ctx.sp_vx = static_cast<float>(dbg.vx_cmd);
    ctx.sp_vy = static_cast<float>(dbg.vy_cmd);
    ctx.sp_vz = static_cast<float>(dbg.vz_cmd);

    ctx.sp_ax = static_cast<float>(dbg.ax_cmd);
    ctx.sp_ay = static_cast<float>(dbg.ay_cmd);
    ctx.sp_az = static_cast<float>(dbg.az_cmd);

    if (cfg_.use_ego_yaw) {
      ctx.sp_yaw = static_cast<float>(ctx.ego_yaw);
    } else {
      ctx.sp_yaw = hold_yaw_;
    }

    last_sp_x_ = ctx.sp_x;
    last_sp_y_ = ctx.sp_y;
    last_sp_z_ = ctx.sp_z;

    dbg.reason = "OK";
    return Result::OK;
  }

  static const char* resultName(Result r)
  {
    switch (r) {
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
  void holdCurrent(Context& ctx)
  {
    ctx.sp_x = ctx.cx();
    ctx.sp_y = ctx.cy();
    ctx.sp_z = ctx.cz();
    ctx.sp_yaw = hold_yaw_;

    clearVelocityAndAcceleration(ctx);

    last_sp_x_ = ctx.sp_x;
    last_sp_y_ = ctx.sp_y;
    last_sp_z_ = ctx.sp_z;
  }

  void clearVelocityAndAcceleration(Context& ctx)
  {
    ctx.sp_vx = 0.0f;
    ctx.sp_vy = 0.0f;
    ctx.sp_vz = 0.0f;

    ctx.sp_ax = 0.0f;
    ctx.sp_ay = 0.0f;
    ctx.sp_az = 0.0f;
  }

  void mapXY(double x_in, double y_in, double& x_out, double& y_out) const
  {
    double ex = x_in;
    double ey = y_in;

    if (cfg_.swap_xy) {
      ex = y_in;
      ey = x_in;
    }

    const double c = std::cos(cfg_.yaw_align_rad);
    const double s = std::sin(cfg_.yaw_align_rad);

    double rx = c * ex - s * ey;
    double ry = s * ex + c * ey;

    rx *= cfg_.x_sign;
    ry *= cfg_.y_sign;

    x_out = rx;
    y_out = ry;
  }

  static void limitXY(double& x, double& y, double max_norm)
  {
    const double n = std::sqrt(x * x + y * y);

    if (n > max_norm && n > 1e-6) {
      const double scale = max_norm / n;
      x *= scale;
      y *= scale;
    }
  }

  static float clampStep(float current, float target, float max_step)
  {
    const float d = target - current;

    if (d > max_step) {
      return current + max_step;
    }

    if (d < -max_step) {
      return current - max_step;
    }

    return target;
  }

  static uint64_t nowUs()
  {
    return static_cast<uint64_t>(
      rclcpp::Clock().now().nanoseconds() / 1000ULL);
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