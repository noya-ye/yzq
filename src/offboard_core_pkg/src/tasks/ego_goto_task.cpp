// 内部会完成：

// 发布 EGO 目标
// → 调用 EgoVelPlanner 跟踪 /position_cmd
// → 判断实际位置是否到达
// → 稳定后返回 SUCCESS
// → Scheduler 自动进入下一个任务
// EgoGotoTask 使用要点
// 1.作用
// 发布 EGO 目标，跟踪 /position_cmd，到达并稳定后返回 SUCCESS。

// 2.节点必须准备

// ego_goal_pub_
// /position_cmd 订阅
// /fastlio2/lio_odom 订阅

// 3.配置目标

// offboard_core_pkg::EgoGotoTask::Config cfg;
// cfg.task_name = "EGO_TO_TARGET";
// cfg.goal_name = "C5";
// cfg.goal_frame = "camera_init";
// cfg.x_rel = 2.25;
// cfg.y_rel = 2.20;
// cfg.height_m = 1.00;
// cfg.yaw_local = M_PI_2;
// cfg.planner = ego_cfg;这里的 ego_cfg 是 EgoVelPlanner::Config 类型，配置 EGO 跟踪参数。

// 4.加入 Scheduler

// sched_.add(std::make_unique<offboard_core_pkg::EgoGotoTask>(
//   get_logger(), get_clock(), ego_goal_pub_, cfg));
// 5.坐标含义
// x_rel、y_rel：相对起飞点的 PX4 local 坐标
// height_m：离地高度
// yaw_local：PX4 local 航向

#include "offboard_core_pkg/tasks/ego_goto_task.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace offboard_core_pkg
{

EgoGotoTask::EgoGotoTask(
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
  const Config& cfg)
: logger_(logger),
  clock_(std::move(clock)),
  goal_pub_(std::move(goal_pub)),
  cfg_(cfg),
  planner_(cfg.planner)
{
}

std::string EgoGotoTask::name() const
{
  return cfg_.task_name;
}

void EgoGotoTask::onEnter(Context& ctx)
{
  started_ = false;
  finished_ = false;
  stable_elapsed_s_ = 0.0;
  republish_elapsed_s_ = 0.0;
  accept_cmd_after_us_ = 0;

  ctx.use_vel_ctrl = true;
  ctx.ego_cmd_valid = false;
  planner_.reset(ctx);

  RCLCPP_WARN(
    logger_, "[%s] enter goal=%s rel=(%.2f %.2f) height=%.2f yaw=%.3f",
    cfg_.task_name.c_str(), cfg_.goal_name.c_str(),
    cfg_.x_rel, cfg_.y_rel, cfg_.height_m, cfg_.yaw_local);
}

ITask::Status EgoGotoTask::tick(Context& ctx, double dt_s)
{
  if (finished_) {
    return Status::SUCCESS;
  }

  const double dt = std::clamp(std::isfinite(dt_s) ? dt_s : 0.0, 0.0, 0.20);

  if (!started_ && !startGoal(ctx)) {
    holdCurrent(ctx);
    return Status::RUNNING;
  }

  republish_elapsed_s_ += dt;
  if (republish_elapsed_s_ >= cfg_.goal_republish_s) {
    publishGoal();
    republish_elapsed_s_ = 0.0;
  }

  if (!ctx.ego_cmd_valid || ctx.ego_cmd_stamp_us < accept_cmd_after_us_) {
    holdCurrent(ctx);
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 500, "[%s] waiting new EGO command", cfg_.task_name.c_str());
    return Status::RUNNING;
  }

  ctx.use_vel_ctrl = true;

  EgoVelPlanner::Debug dbg;
  const EgoVelPlanner::Result result = planner_.plan(ctx, &dbg);
  ctx.sp_yaw = static_cast<float>(cfg_.yaw_local);

  if (result != EgoVelPlanner::Result::OK) {
    stable_elapsed_s_ = 0.0;

    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 500,
      "[%s] hold result=%s reason=%s age=(cmd %.2f odom %.2f)",
      cfg_.task_name.c_str(), EgoVelPlanner::resultName(result),
      dbg.reason.c_str(), dbg.cmd_age_s, dbg.odom_age_s);

    return Status::RUNNING;
  }

  const double err_xy = std::hypot(
    static_cast<double>(ctx.cx()) - target_local_x_,
    static_cast<double>(ctx.cy()) - target_local_y_);
  const double err_z = std::fabs(static_cast<double>(ctx.cz()) - target_local_z_);
  const double vxy = std::hypot(ctx.local_pos.vx, ctx.local_pos.vy);
  const double vz = std::fabs(ctx.local_pos.vz);

  const bool stable =
    err_xy <= cfg_.arrive_xy_m &&
    err_z <= cfg_.arrive_z_m &&
    vxy <= cfg_.stable_vxy_mps &&
    vz <= cfg_.stable_vz_mps;

  stable_elapsed_s_ = stable ? stable_elapsed_s_ + dt : 0.0;

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 500,
    "[%s] goal=%s err=(xy %.3f z %.3f) speed=(xy %.3f z %.3f) stable=%.2f/%.2f",
    cfg_.task_name.c_str(), cfg_.goal_name.c_str(),
    err_xy, err_z, vxy, vz, stable_elapsed_s_, cfg_.stable_required_s);

  if (stable_elapsed_s_ < cfg_.stable_required_s) {
    return Status::RUNNING;
  }

  finished_ = true;
  holdCurrent(ctx);

  RCLCPP_WARN(
    logger_, "[%s] reached %s local=(%.2f %.2f %.2f)",
    cfg_.task_name.c_str(), cfg_.goal_name.c_str(),
    target_local_x_, target_local_y_, target_local_z_);

  return Status::SUCCESS;
}

void EgoGotoTask::onExit(Context& ctx)
{
  holdCurrent(ctx);
}

bool EgoGotoTask::startGoal(Context& ctx)
{
  if (!ctx.home_inited || !ctx.pos_valid() || !ctx.ego_odom_valid) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[%s] waiting home/position/EGO odom: home=%d pos=%d ego_odom=%d",
      cfg_.task_name.c_str(), ctx.home_inited ? 1 : 0,
      ctx.pos_valid() ? 1 : 0, ctx.ego_odom_valid ? 1 : 0);
    return false;
  }

  target_local_x_ = static_cast<double>(ctx.home_x) + cfg_.x_rel;
  target_local_y_ = static_cast<double>(ctx.home_y) + cfg_.y_rel;
  target_local_z_ = static_cast<double>(ctx.home_z) - cfg_.height_m;

  const double local_dx = target_local_x_ - static_cast<double>(ctx.cx());
  const double local_dy = target_local_y_ - static_cast<double>(ctx.cy());
  const double local_dz = target_local_z_ - static_cast<double>(ctx.cz());

  double ego_dx = 0.0;
  double ego_dy = 0.0;
  inverseMapXY(local_dx, local_dy, ego_dx, ego_dy);

  target_ego_x_ = ctx.ego_odom_x + ego_dx;
  target_ego_y_ = ctx.ego_odom_y + ego_dy;
  target_ego_z_ = ctx.ego_odom_z - local_dz;  // EGO z向上，PX4 NED z向下

  started_ = true;
  stable_elapsed_s_ = 0.0;
  republish_elapsed_s_ = 0.0;

  ctx.ego_cmd_valid = false;
  accept_cmd_after_us_ =
    nowUs() + static_cast<uint64_t>(std::max(0.0, cfg_.cmd_guard_s) * 1e6);

  planner_.reset(ctx);
  publishGoal();

  RCLCPP_WARN(
    logger_, "[%s] publish %s local=(%.2f %.2f %.2f) ego=(%.2f %.2f %.2f)",
    cfg_.task_name.c_str(), cfg_.goal_name.c_str(),
    target_local_x_, target_local_y_, target_local_z_,
    target_ego_x_, target_ego_y_, target_ego_z_);

  return true;
}

void EgoGotoTask::publishGoal()
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = clock_->now();
  goal.header.frame_id = cfg_.goal_frame;
  goal.pose.position.x = target_ego_x_;
  goal.pose.position.y = target_ego_y_;
  goal.pose.position.z = target_ego_z_;
  goal.pose.orientation.w = 1.0;
  goal_pub_->publish(goal);
}

void EgoGotoTask::holdCurrent(Context& ctx)
{
  ctx.use_vel_ctrl = false;
  ctx.sp_x = ctx.cx();
  ctx.sp_y = ctx.cy();
  ctx.sp_z = ctx.cz();
  ctx.sp_yaw = static_cast<float>(cfg_.yaw_local);
  ctx.sp_vx = ctx.sp_vy = ctx.sp_vz = 0.0f;
  ctx.sp_ax = ctx.sp_ay = ctx.sp_az = 0.0f;
}

void EgoGotoTask::inverseMapXY(
  double local_x, double local_y, double& ego_x, double& ego_y) const
{
  const double sx = std::fabs(cfg_.planner.x_sign) > 1e-6 ? cfg_.planner.x_sign : 1.0;
  const double sy = std::fabs(cfg_.planner.y_sign) > 1e-6 ? cfg_.planner.y_sign : 1.0;
  const double rx = local_x / sx;
  const double ry = local_y / sy;
  const double c = std::cos(cfg_.planner.yaw_align_rad);
  const double s = std::sin(cfg_.planner.yaw_align_rad);
  const double ex = c * rx + s * ry;
  const double ey = -s * rx + c * ry;

  if (cfg_.planner.swap_xy) {
    ego_x = ey;
    ego_y = ex;
  } else {
    ego_x = ex;
    ego_y = ey;
  }
}

uint64_t EgoGotoTask::nowUs()
{
  return static_cast<uint64_t>(rclcpp::Clock().now().nanoseconds() / 1000ULL);
}

}  // namespace offboard_core_pkg