#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/planners/ego_vel_planner.hpp"

namespace offboard_core_pkg
{

class EgoVelFollowTask : public ITask
{
public:
  using Config = EgoVelPlanner::Config;

  EgoVelFollowTask(rclcpp::Logger logger)
  : logger_(logger),
    planner_()
  {}

  EgoVelFollowTask(rclcpp::Logger logger, const Config& cfg)
  : logger_(logger),
    planner_(cfg)
  {}

  std::string name() const override
  {
    return "EGO_VEL_FOLLOW";
  }

  void onEnter(Context& ctx) override
  {
    ctx.use_vel_ctrl = true;
    planner_.reset(ctx);

    const auto& cfg = planner_.config();

    RCLCPP_WARN(
      logger_,
      "[EgoVelFollowTask] enter planner-mode | px4_cur=(%.3f %.3f %.3f), "
      "sign=(%.0f %.0f), swap_xy=%d, yaw_align=%.3f, "
      "kp_xy=%.2f, kp_vel_xy=%.2f, vel_ff=%.2f, acc_ff=%.2f",
      ctx.cx(), ctx.cy(), ctx.cz(),
      cfg.x_sign, cfg.y_sign,
      static_cast<int>(cfg.swap_xy),
      cfg.yaw_align_rad,
      cfg.kp_xy,
      cfg.kp_vel_xy,
      cfg.vel_ff_scale,
      cfg.acc_ff_scale);
  }

  ITask::Status tick(Context& ctx, double) override
  {
    ctx.use_vel_ctrl = true;

    EgoVelPlanner::Debug dbg;
    const EgoVelPlanner::Result result = planner_.plan(ctx, &dbg);

    if (result != EgoVelPlanner::Result::OK) {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        500,
        "[EgoVelFollowTask] planner hold | result=%s reason=%s "
        "age=(cmd %.2f, odom %.2f), hold=(%.3f %.3f %.3f)",
        EgoVelPlanner::resultName(result),
        dbg.reason.c_str(),
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
      "ego_cmd=(%.3f %.3f %.3f), ego_odom=(%.3f %.3f %.3f), "
      "d_ego=(%.3f %.3f %.3f), mapped_err=(%.3f %.3f), "
      "pos_sp=(%.3f %.3f %.3f), vel_sp=(%.3f %.3f %.3f), acc_sp=(%.3f %.3f %.3f), "
      "px4_cur=(%.3f %.3f %.3f), age=(cmd %.2f, odom %.2f)",
      ctx.ego_x, ctx.ego_y, ctx.ego_z,
      ctx.ego_odom_x, ctx.ego_odom_y, ctx.ego_odom_z,
      dbg.dx_ego, dbg.dy_ego, dbg.dz_ego,
      dbg.ex_map, dbg.ey_map,
      ctx.sp_x, ctx.sp_y, ctx.sp_z,
      ctx.sp_vx, ctx.sp_vy, ctx.sp_vz,
      ctx.sp_ax, ctx.sp_ay, ctx.sp_az,
      ctx.cx(), ctx.cy(), ctx.cz(),
      dbg.cmd_age_s,
      dbg.odom_age_s);

    return Status::RUNNING;
  }

private:
  rclcpp::Logger logger_;
  EgoVelPlanner planner_;
};

}  // namespace offboard_core_pkg