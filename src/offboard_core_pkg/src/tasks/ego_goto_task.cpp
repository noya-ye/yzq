// 内部会完成：
//
// 发布 EGO 目标
// → 调用 EgoVelPlanner 跟踪 /position_cmd
// → 使用 PX4 位置主控 + 速度/加速度前馈
// → 判断实际位置是否到达
// → 稳定后返回 SUCCESS
// → Scheduler 自动进入下一个任务
//
// EgoGotoTask 使用要点：
//
// 1. 作用
// 发布 EGO 目标，跟踪 /position_cmd，到达并稳定后返回 SUCCESS。
//
// 2. 节点必须准备
//
// ego_goal_pub_
// /position_cmd 订阅
// /fastlio2/lio_odom 订阅
//
// 3. 配置目标
//
// offboard_core_pkg::EgoGotoTask::Config cfg;
// cfg.task_name = "EGO_TO_TARGET";
// cfg.goal_name = "C5";
// cfg.goal_frame = "camera_init";
// cfg.x_rel = 2.25;
// cfg.y_rel = 2.20;
// cfg.height_m = 1.00;
// cfg.yaw_local = M_PI_2;
// cfg.planner = ego_cfg;
//
// ego_cfg 类型为 EgoVelPlanner::Config。
//
// 4. 加入 Scheduler
//
// sched_.add(std::make_unique<offboard_core_pkg::EgoGotoTask>(
//   get_logger(),
//   get_clock(),
//   ego_goal_pub_,
//   cfg));
//
// 5. 坐标含义
//
// x_rel、y_rel：相对起飞点的 PX4 local 坐标
// height_m：相对地面的高度
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

  // EGO 轨迹使用 PX4 位置控制器。
  //
  // position     = 主控制目标
  // velocity     = 可选速度前馈
  // acceleration = 可选加速度前馈
  ctx.use_vel_ctrl = false;

  ctx.use_trajectory_ff =
    cfg_.planner.use_velocity_ff ||
    cfg_.planner.use_acceleration_ff;

  // 丢弃进入任务前遗留的旧 PositionCommand。
  ctx.ego_cmd_valid = false;

  planner_.reset(ctx);

  RCLCPP_WARN(
    logger_,
    "[%s] enter | goal=%s rel=(%.2f %.2f) height=%.2f yaw=%.3f "
    "mode=(position 1 trajectory_ff %d) "
    "vel_ff=(enable %d scale %.2f) "
    "acc_ff=(enable %d scale %.2f)",
    cfg_.task_name.c_str(),
    cfg_.goal_name.c_str(),
    cfg_.x_rel,
    cfg_.y_rel,
    cfg_.height_m,
    cfg_.yaw_local,
    static_cast<int>(ctx.use_trajectory_ff),
    static_cast<int>(cfg_.planner.use_velocity_ff),
    cfg_.planner.vel_ff_scale,
    static_cast<int>(cfg_.planner.use_acceleration_ff),
    cfg_.planner.acc_ff_scale);
}

ITask::Status EgoGotoTask::tick(
  Context& ctx,
  double dt_s)
{
  if (finished_) {
    return Status::SUCCESS;
  }

  const double dt = std::clamp(
    std::isfinite(dt_s) ? dt_s : 0.0,
    0.0,
    0.20);

  // ============================================================
  // 1. 首次进入时计算并发布目标
  // ============================================================
  if (!started_) {
    if (!startGoal(ctx)) {
      holdCurrent(ctx);
      return Status::RUNNING;
    }
  }

  // ============================================================
  // 2. 周期性重新发布目标
  //
  // goal_republish_s <= 0 时不周期重发。
  // ============================================================
  if (cfg_.goal_republish_s > 0.0) {
    republish_elapsed_s_ += dt;

    if (republish_elapsed_s_ >= cfg_.goal_republish_s) {
      publishGoal();
      republish_elapsed_s_ = 0.0;
    }
  }

  // ============================================================
  // 3. 等待新目标对应的 PositionCommand
  //
  // 不允许使用进入任务前，或者目标发布保护时间内的旧命令。
  // ============================================================
  if (!ctx.ego_cmd_valid ||
      ctx.ego_cmd_stamp_us < accept_cmd_after_us_) {
    stable_elapsed_s_ = 0.0;

    holdCurrent(ctx);

    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      500,
      "[%s] waiting new EGO command | "
      "valid=%d cmd_stamp=%llu accept_after=%llu",
      cfg_.task_name.c_str(),
      static_cast<int>(ctx.ego_cmd_valid),
      static_cast<unsigned long long>(ctx.ego_cmd_stamp_us),
      static_cast<unsigned long long>(accept_cmd_after_us_));

    return Status::RUNNING;
  }

  // ============================================================
  // 4. 开启位置主控及轨迹前馈
  //
  // 必须在每一拍重新设置，避免其他任务残留的控制模式影响。
  // ============================================================
  ctx.use_vel_ctrl = false;

  ctx.use_trajectory_ff =
    cfg_.planner.use_velocity_ff ||
    cfg_.planner.use_acceleration_ff;

  EgoVelPlanner::Debug dbg;

  const EgoVelPlanner::Result result =
    planner_.plan(ctx, &dbg);

  // 本任务使用配置指定的 PX4 local yaw。
  ctx.sp_yaw =
    static_cast<float>(cfg_.yaw_local);

  // ============================================================
  // 5. 规划器异常时悬停
  //
  // EgoVelPlanner 内部应当：
  //
  //   use_vel_ctrl      = false
  //   use_trajectory_ff = false
  //   position          = 当前PX4位置
  //
  // 防止旧速度或旧加速度继续发送。
  // ============================================================
  if (result != EgoVelPlanner::Result::OK) {
    stable_elapsed_s_ = 0.0;

    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      500,
      "[%s] hold | result=%s reason=%s "
      "mode=(position 1 trajectory_ff %d) "
      "age=(cmd %.2f odom %.2f) "
      "hold=(%.3f %.3f %.3f)",
      cfg_.task_name.c_str(),
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

  // ============================================================
  // 6. 使用PX4实际位置判断是否到达
  // ============================================================
  const double err_xy = std::hypot(
    static_cast<double>(ctx.cx()) - target_local_x_,
    static_cast<double>(ctx.cy()) - target_local_y_);

  const double err_z = std::fabs(
    static_cast<double>(ctx.cz()) - target_local_z_);

  const double vxy = std::hypot(
    static_cast<double>(ctx.local_pos.vx),
    static_cast<double>(ctx.local_pos.vy));

  const double vz =
    std::fabs(static_cast<double>(ctx.local_pos.vz));

  const bool stable =
    err_xy <= cfg_.arrive_xy_m &&
    err_z <= cfg_.arrive_z_m &&
    vxy <= cfg_.stable_vxy_mps &&
    vz <= cfg_.stable_vz_mps;

  if (stable) {
    stable_elapsed_s_ += dt;
  } else {
    stable_elapsed_s_ = 0.0;
  }

  RCLCPP_INFO_THROTTLE(
    logger_,
    *clock_,
    500,
    "[%s] goal=%s "
    "err=(xy %.3f z %.3f) "
    "speed=(xy %.3f z %.3f) "
    "stable=%.2f/%.2f "
    "mode=(position 1 trajectory_ff %d) "
    "sp_pos=(%.3f %.3f %.3f) "
    "sp_vel=(%.3f %.3f %.3f) "
    "sp_acc=(%.3f %.3f %.3f)",
    cfg_.task_name.c_str(),
    cfg_.goal_name.c_str(),
    err_xy,
    err_z,
    vxy,
    vz,
    stable_elapsed_s_,
    cfg_.stable_required_s,
    static_cast<int>(ctx.use_trajectory_ff),
    ctx.sp_x,
    ctx.sp_y,
    ctx.sp_z,
    ctx.sp_vx,
    ctx.sp_vy,
    ctx.sp_vz,
    ctx.sp_ax,
    ctx.sp_ay,
    ctx.sp_az);

  if (stable_elapsed_s_ < cfg_.stable_required_s) {
    return Status::RUNNING;
  }

  // ============================================================
  // 7. 到达目标
  // ============================================================
  finished_ = true;

  // 到达后立即关闭轨迹前馈，并锁定当前位置。
  holdCurrent(ctx);

  RCLCPP_WARN(
    logger_,
    "[%s] reached %s | local=(%.2f %.2f %.2f)",
    cfg_.task_name.c_str(),
    cfg_.goal_name.c_str(),
    target_local_x_,
    target_local_y_,
    target_local_z_);

  return Status::SUCCESS;
}

void EgoGotoTask::onExit(Context& ctx)
{
  // 离开任务后恢复普通位置悬停，
  // 防止下一个任务收到本任务遗留的速度和加速度前馈。
  holdCurrent(ctx);

  RCLCPP_WARN(
    logger_,
    "[%s] exit | trajectory feedforward disabled",
    cfg_.task_name.c_str());
}

bool EgoGotoTask::startGoal(Context& ctx)
{
  // ============================================================
  // 1. 等待必需状态
  // ============================================================
  if (!ctx.home_inited ||
      !ctx.pos_valid() ||
      !ctx.ego_odom_valid) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      1000,
      "[%s] waiting home/position/EGO odom | "
      "home=%d pos=%d ego_odom=%d",
      cfg_.task_name.c_str(),
      static_cast<int>(ctx.home_inited),
      static_cast<int>(ctx.pos_valid()),
      static_cast<int>(ctx.ego_odom_valid));

    return false;
  }

  // ============================================================
  // 2. 计算PX4 local目标
  // ============================================================
  target_local_x_ =
    static_cast<double>(ctx.home_x) +
    cfg_.x_rel;

  target_local_y_ =
    static_cast<double>(ctx.home_y) +
    cfg_.y_rel;

  // PX4 NED中，z越小表示高度越高。
  target_local_z_ =
    static_cast<double>(ctx.home_z) -
    cfg_.height_m;

  // ============================================================
  // 3. 计算当前PX4位置到目标的相对位移
  // ============================================================
  const double local_dx =
    target_local_x_ -
    static_cast<double>(ctx.cx());

  const double local_dy =
    target_local_y_ -
    static_cast<double>(ctx.cy());

  const double local_dz =
    target_local_z_ -
    static_cast<double>(ctx.cz());

  // ============================================================
  // 4. 将PX4 local位移反向转换到EGO世界系
  // ============================================================
  double ego_dx = 0.0;
  double ego_dy = 0.0;

  inverseMapXY(
    local_dx,
    local_dy,
    ego_dx,
    ego_dy);

  // 使用FAST-LIO当前里程计作为相对转换基准，
  // 不要求PX4 local原点与FAST-LIO世界系原点相同。
  target_ego_x_ =
    ctx.ego_odom_x + ego_dx;

  target_ego_y_ =
    ctx.ego_odom_y + ego_dy;

  // EGO z向上，PX4 NED z向下。
  target_ego_z_ =
    ctx.ego_odom_z - local_dz;

  // ============================================================
  // 5. 初始化本次目标状态
  // ============================================================
  started_ = true;

  stable_elapsed_s_ = 0.0;
  republish_elapsed_s_ = 0.0;

  // 丢弃旧的PositionCommand。
  ctx.ego_cmd_valid = false;

  const double guard_s =
    std::max(0.0, cfg_.cmd_guard_s);

  accept_cmd_after_us_ =
    nowUs() +
    static_cast<uint64_t>(guard_s * 1e6);

  // 位置主控 + 可选轨迹前馈。
  ctx.use_vel_ctrl = false;

  ctx.use_trajectory_ff =
    cfg_.planner.use_velocity_ff ||
    cfg_.planner.use_acceleration_ff;

  planner_.reset(ctx);

  publishGoal();

  RCLCPP_WARN(
    logger_,
    "[%s] publish %s | "
    "local=(%.2f %.2f %.2f) "
    "ego=(%.2f %.2f %.2f) "
    "cmd_guard=%.2f",
    cfg_.task_name.c_str(),
    cfg_.goal_name.c_str(),
    target_local_x_,
    target_local_y_,
    target_local_z_,
    target_ego_x_,
    target_ego_y_,
    target_ego_z_,
    guard_s);

  return true;
}

void EgoGotoTask::publishGoal()
{
  geometry_msgs::msg::PoseStamped goal;

  goal.header.stamp =
    clock_->now();

  goal.header.frame_id =
    cfg_.goal_frame;

  goal.pose.position.x =
    target_ego_x_;

  goal.pose.position.y =
    target_ego_y_;

  goal.pose.position.z =
    target_ego_z_;

  // 当前规划器主要使用位置；
  // yaw由PX4 setpoint中的cfg_.yaw_local控制。
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = 0.0;
  goal.pose.orientation.w = 1.0;

  goal_pub_->publish(goal);
}

void EgoGotoTask::holdCurrent(Context& ctx)
{
  // 普通位置控制。
  ctx.use_vel_ctrl = false;

  // 关闭速度、加速度前馈。
  ctx.use_trajectory_ff = false;

  ctx.sp_x = ctx.cx();
  ctx.sp_y = ctx.cy();
  ctx.sp_z = ctx.cz();

  ctx.sp_yaw =
    static_cast<float>(cfg_.yaw_local);

  // 这些字段不会在use_trajectory_ff=false时发给PX4，
  // 但仍清零，防止后续任务误启用旧前馈。
  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  ctx.sp_ax = 0.0f;
  ctx.sp_ay = 0.0f;
  ctx.sp_az = 0.0f;
}

void EgoGotoTask::inverseMapXY(
  double local_x,
  double local_y,
  double& ego_x,
  double& ego_y) const
{
  // ============================================================
  // 正向mapXY的顺序：
  //
  // 1. 可选swap
  // 2. yaw旋转
  // 3. x_sign / y_sign
  //
  // 逆变换按相反顺序执行：
  //
  // 1. 解除sign
  // 2. 反向yaw旋转
  // 3. 解除swap
  // ============================================================

  const double x_sign =
    std::fabs(cfg_.planner.x_sign) > 1e-6
      ? cfg_.planner.x_sign
      : 1.0;

  const double y_sign =
    std::fabs(cfg_.planner.y_sign) > 1e-6
      ? cfg_.planner.y_sign
      : 1.0;

  // 解除方向符号。
  const double rotated_x =
    local_x / x_sign;

  const double rotated_y =
    local_y / y_sign;

  // 反向旋转：R(-yaw)。
  const double c =
    std::cos(cfg_.planner.yaw_align_rad);

  const double s =
    std::sin(cfg_.planner.yaw_align_rad);

  const double mapped_x =
    c * rotated_x +
    s * rotated_y;

  const double mapped_y =
    -s * rotated_x +
    c * rotated_y;

  // 解除swap。
  if (cfg_.planner.swap_xy) {
    ego_x = mapped_y;
    ego_y = mapped_x;
  } else {
    ego_x = mapped_x;
    ego_y = mapped_y;
  }
}

uint64_t EgoGotoTask::nowUs() const
{
  return static_cast<uint64_t>(
    clock_->now().nanoseconds() /
    1000ULL);
}

}  // namespace offboard_core_pkg