#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/planners/ego_vel_planner.hpp"

namespace offboard_core_pkg
{

/*
 * EGO连续轨迹跟踪任务
 *
 * 正常运行时：
 *
 *   ctx.use_vel_ctrl      = false
 *   ctx.use_trajectory_ff = true/false
 *
 * PX4使用位置控制器：
 *
 *   position     = 主控制目标
 *   velocity     = 可选速度前馈
 *   acceleration = 可选加速度前馈
 *
 * 不再使用纯速度控制模式。
 */
class EgoVelFollowTask : public ITask
{
public:
  using Config = EgoVelPlanner::Config;

  EgoVelFollowTask(rclcpp::Logger logger)
  : logger_(logger),
    planner_()
  {
  }

  EgoVelFollowTask(
    rclcpp::Logger logger,
    const Config& cfg)
  : logger_(logger),
    planner_(cfg)
  {
  }

  std::string name() const override
  {
    return "EGO_VEL_FOLLOW";
  }

  void onEnter(Context& ctx) override
  {
    const auto& cfg = planner_.config();

    // EGO轨迹采用PX4位置主控制。
    ctx.use_vel_ctrl = false;

    // 只有至少启用一种前馈时，才打开轨迹前馈模式。
    ctx.use_trajectory_ff =
      cfg.use_velocity_ff ||
      cfg.use_acceleration_ff;

    planner_.reset(ctx);

    RCLCPP_WARN(
      logger_,
      "[EgoVelFollowTask] enter trajectory-mode | "
      "px4_cur=(%.3f %.3f %.3f), "
      "position_ctrl=1 trajectory_ff=%d, "
      "sign=(%.0f %.0f), swap_xy=%d, yaw_align=%.3f, "
      "kp_xy=%.2f kp_z=%.2f, "
      "vel_ff=(enable %d scale %.2f), "
      "acc_ff=(enable %d scale %.2f)",
      ctx.cx(),
      ctx.cy(),
      ctx.cz(),
      static_cast<int>(ctx.use_trajectory_ff),
      cfg.x_sign,
      cfg.y_sign,
      static_cast<int>(cfg.swap_xy),
      cfg.yaw_align_rad,
      cfg.kp_xy,
      cfg.kp_z,
      static_cast<int>(cfg.use_velocity_ff),
      cfg.vel_ff_scale,
      static_cast<int>(cfg.use_acceleration_ff),
      cfg.acc_ff_scale);
  }

  ITask::Status tick(
    Context& ctx,
    double dt_s) override
  {
    (void)dt_s;

    const auto& cfg = planner_.config();

    // 每一拍明确使用位置主控制，避免被上一个任务残留状态影响。
    ctx.use_vel_ctrl = false;

    // planner执行失败时，会在内部把该开关重新关闭。
    ctx.use_trajectory_ff =
      cfg.use_velocity_ff ||
      cfg.use_acceleration_ff;

    EgoVelPlanner::Debug dbg;

    const EgoVelPlanner::Result result =
      planner_.plan(ctx, &dbg);

    if (result != EgoVelPlanner::Result::OK) {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        500,
        "[EgoVelFollowTask] planner hold | "
        "result=%s reason=%s "
        "mode=(position 1 trajectory_ff %d) "
        "age=(cmd %.2f, odom %.2f), "
        "hold=(%.3f %.3f %.3f)",
        EgoVelPlanner::resultName(result),
        dbg.reason.c_str(),
        static_cast<int>(ctx.use_trajectory_ff),
        dbg.cmd_age_s,
        dbg.odom_age_s,
        ctx.sp_x,
        ctx.sp_y,
        ctx.sp_z);

      return Status::RUNNING;
    }

    RCLCPP_WARN_THROTTLE(
      logger_,
      *rclcpp::Clock::make_shared(),
      500,
      "[EgoVelFollowTask] planner ok | "
      "mode=(position 1 trajectory_ff %d), "
      "ego_cmd=(%.3f %.3f %.3f), "
      "ego_odom=(%.3f %.3f %.3f), "
      "d_ego=(%.3f %.3f %.3f), "
      "mapped_err=(%.3f %.3f), "
      "pos_sp=(%.3f %.3f %.3f), "
      "vel_ff=(%.3f %.3f %.3f), "
      "acc_ff=(%.3f %.3f %.3f), "
      "px4_cur=(%.3f %.3f %.3f), "
      "age=(cmd %.2f, odom %.2f)",
      static_cast<int>(ctx.use_trajectory_ff),
      ctx.ego_x,
      ctx.ego_y,
      ctx.ego_z,
      ctx.ego_odom_x,
      ctx.ego_odom_y,
      ctx.ego_odom_z,
      dbg.dx_ego,
      dbg.dy_ego,
      dbg.dz_ego,
      dbg.ex_map,
      dbg.ey_map,
      ctx.sp_x,
      ctx.sp_y,
      ctx.sp_z,
      ctx.sp_vx,
      ctx.sp_vy,
      ctx.sp_vz,
      ctx.sp_ax,
      ctx.sp_ay,
      ctx.sp_az,
      ctx.cx(),
      ctx.cy(),
      ctx.cz(),
      dbg.cmd_age_s,
      dbg.odom_age_s);

    return Status::RUNNING;
  }

  void onExit(Context& ctx) override
  {
    // 离开EGO任务后恢复普通位置控制，
    // 防止下一个任务继续发送旧速度和旧加速度。
    ctx.use_vel_ctrl = false;
    ctx.use_trajectory_ff = false;

    ctx.sp_vx = 0.0f;
    ctx.sp_vy = 0.0f;
    ctx.sp_vz = 0.0f;

    ctx.sp_ax = 0.0f;
    ctx.sp_ay = 0.0f;
    ctx.sp_az = 0.0f;

    RCLCPP_WARN(
      logger_,
      "[EgoVelFollowTask] exit | "
      "trajectory feedforward disabled");
  }

private:
  rclcpp::Logger logger_;
  EgoVelPlanner planner_;
};

}  // namespace offboard_core_pkg