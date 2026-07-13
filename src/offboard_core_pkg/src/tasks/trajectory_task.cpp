//轨迹飞行+中断设置
//  ctx.front_pre_align_request = true;与外界通信，中断标志位。下次如果需要中断接另外的程序，只需要把那个程序在on_timer设置成true则开启

// InputWaypoint 和 Waypoint 的核心区别是：

// InputWaypoint 是用户输入的原始航点；Waypoint 是程序解析、转换后真正用于飞行的内部航点。
// #include "offboard_core_pkg/tasks/trajectory_task.hpp"，使用时只需要输入inputwaypoint就行了
//绝对坐标 ABSOLUTE_LOCAL时：输入的坐标直接就是飞机最终要到达的局部坐标。飞机打开雷达处为原点构建的坐标系，与进入任务时飞机的位置无关
//相对坐标 RELATIVE_TO_ENTER：输入航点当作相对于进入位置的偏移量

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include "offboard_core_pkg/tasks/trajectory_task.hpp"

TrajectoryTask::TrajectoryTask(
  rclcpp::Logger logger,
  const Config& cfg)
: logger_(logger),
  cfg_(cfg)
{
}

std::string TrajectoryTask::name() const
{
  return "TRAJECTORY";
}

void TrajectoryTask::onEnter(Context& ctx)
{
  waypoints_.clear();

  phase_ = Phase::FINISHED;
  index_ = 0;
  hover_elapsed_s_ = 0.0;

  waiting_pre_align_ = false;
  paused_for_pre_align_ = false;
  pre_align_waypoint_index_ = 0;

  ctx.front_pre_align_request = false;

  if (!ctx.pos_valid()) {
    RCLCPP_ERROR(
      logger_,
      "[TRAJECTORY] enter failed: local position invalid");

    phase_ = Phase::FAILED;
    return;
  }

  if (!ctx.has_attitude ||
      !std::isfinite(ctx.yaw))
  {
    RCLCPP_ERROR(
      logger_,
      "[TRAJECTORY] enter failed: attitude invalid "
      "has_attitude=%d yaw=%.3f",
      ctx.has_attitude ? 1 : 0,
      ctx.yaw);

    phase_ = Phase::FAILED;
    return;
  }

  if (!std::isfinite(cfg_.max_step_m) ||
      cfg_.max_step_m <= 0.0)
  {
    RCLCPP_ERROR(
      logger_,
      "[TRAJECTORY] invalid max_step_m=%.3f",
      cfg_.max_step_m);

    phase_ = Phase::FAILED;
    return;
  }

  if (!std::isfinite(cfg_.arrive_xy_m) ||
      !std::isfinite(cfg_.arrive_z_m) ||
      cfg_.arrive_xy_m <= 0.0 ||
      cfg_.arrive_z_m <= 0.0)
  {
    RCLCPP_ERROR(
      logger_,
      "[TRAJECTORY] invalid arrive tolerance "
      "xy=%.3f z=%.3f",
      cfg_.arrive_xy_m,
      cfg_.arrive_z_m);

    phase_ = Phase::FAILED;
    return;
  }

  enter_x_ = static_cast<double>(ctx.cx());
  enter_y_ = static_cast<double>(ctx.cy());
  enter_z_ = static_cast<double>(ctx.cz());

  cmd_x_ = enter_x_;
  cmd_y_ = enter_y_;
  cmd_z_ = enter_z_;

  yaw_hold_ = static_cast<double>(ctx.yaw);

  buildWaypoints();

  if (phase_ == Phase::FAILED) {
    return;
  }

  if (waypoints_.empty()) {
    RCLCPP_WARN(
      logger_,
      "[TRAJECTORY] no waypoints, finish directly");

    phase_ = Phase::FINISHED;
    return;
  }

  phase_ = Phase::MOVING;
  index_ = 0;

  publishSetpoint(ctx);

  RCLCPP_WARN(
    logger_,
    "[TRAJECTORY] enter "
    "count=%zu coord_mode=%s tsp=%d "
    "origin=(%.3f %.3f %.3f) yaw=%.3f",
    waypoints_.size(),
    cfg_.coord_mode == CoordMode::ABSOLUTE_LOCAL ?
      "ABSOLUTE_LOCAL" :
      "RELATIVE_TO_ENTER",
    cfg_.enable_tsp_sort ? 1 : 0,
    enter_x_,
    enter_y_,
    enter_z_,
    yaw_hold_);

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const Waypoint& wp = waypoints_[i];

    RCLCPP_INFO(
      logger_,
      "[TRAJECTORY] wp[%zu] input=%zu "
      "pos=(%.3f %.3f %.3f) "
      "hover=%d hover_s=%.2f yaw=%s %.3f "
      "pre_align=%d",
      i,
      wp.input_index,
      wp.x,
      wp.y,
      wp.z,
      wp.hover_after ? 1 : 0,
      wp.hover_s,
      wp.use_yaw ? "CUSTOM" : "HOLD",
      waypointYaw(wp),
      wp.pre_align_after ? 1 : 0);
  }
}

ITask::Status TrajectoryTask::tick(
  Context& ctx,
  double dt_s)
{
  if (phase_ == Phase::FAILED) {
    return Status::FAILURE;
  }

  if (phase_ == Phase::FINISHED) {
    return Status::SUCCESS;
  }

  if (!ctx.pos_valid()) {
    RCLCPP_ERROR(
      logger_,
      "[TRAJECTORY] local position became invalid");

    phase_ = Phase::FAILED;
    return Status::FAILURE;
  }

  if (index_ >= waypoints_.size()) {
    phase_ = Phase::FINISHED;
    return Status::SUCCESS;
  }

  double safe_dt = dt_s;

  if (!std::isfinite(safe_dt) || safe_dt < 0.0) {
    safe_dt = 0.0;
  }

  safe_dt = std::min(safe_dt, 0.2);

  /*
   * 已经发出前置矫正请求，但 Scheduler 可能要到下一次
   * on_timer 才执行 interrupt。
   *
   * 这段期间保持固定位置，并持续保留 request=true，
   * 以便 interrupt 失败时下一周期可以继续重试。
   */
  if (waiting_pre_align_) {
    ctx.front_pre_align_request = true;

    cmd_x_ = interrupt_hold_x_;
    cmd_y_ = interrupt_hold_y_;
    cmd_z_ = interrupt_hold_z_;

    publishSetpoint(ctx);

    return Status::RUNNING;
  }

  const Waypoint& wp = waypoints_[index_];

  if (phase_ == Phase::MOVING) {
    moveCommandToward(wp);
    publishSetpoint(ctx);

    if (!arrived(ctx, wp)) {
      return Status::RUNNING;
    }

    // 到点后把 setpoint 固定到原航点
    cmd_x_ = wp.x;
    cmd_y_ = wp.y;
    cmd_z_ = wp.z;

    publishSetpoint(ctx);

    RCLCPP_INFO(
      logger_,
      "[TRAJECTORY] arrived wp=%zu input=%zu "
      "actual=(%.3f %.3f %.3f) target=(%.3f %.3f %.3f)",
      index_,
      wp.input_index,
      ctx.cx(),
      ctx.cy(),
      ctx.cz(),
      wp.x,
      wp.y,
      wp.z);

    /*
     * 如果配置了停留，先停稳，再触发前置视觉矫正。
     *
     * 这样流程是：
     * 到达粗定位点 -> 停留稳定 -> 前置矫正
     */
    if (wp.hover_after && wp.hover_s > 0.0) {
      phase_ = Phase::HOVERING;
      hover_elapsed_s_ = 0.0;

      RCLCPP_INFO(
        logger_,
        "[TRAJECTORY] hover start wp=%zu duration=%.2fs",
        index_,
        wp.hover_s);

      return Status::RUNNING;
    }

    completeCurrentWaypoint(ctx);

    return phase_ == Phase::FINISHED ?
      Status::SUCCESS :
      Status::RUNNING;
  }

  if (phase_ == Phase::HOVERING) {
    // 停留期间始终固定在原航点
    cmd_x_ = wp.x;
    cmd_y_ = wp.y;
    cmd_z_ = wp.z;

    publishSetpoint(ctx);

    hover_elapsed_s_ += safe_dt;

    if (hover_elapsed_s_ < wp.hover_s) {
      return Status::RUNNING;
    }

    RCLCPP_INFO(
      logger_,
      "[TRAJECTORY] hover finished wp=%zu elapsed=%.2fs",
      index_,
      hover_elapsed_s_);

    hover_elapsed_s_ = 0.0;

    completeCurrentWaypoint(ctx);

    return phase_ == Phase::FINISHED ?
      Status::SUCCESS :
      Status::RUNNING;
  }

  return Status::RUNNING;
}

void TrajectoryTask::onExit(Context& ctx)
{
  ctx.front_pre_align_request = false;

  waiting_pre_align_ = false;
  paused_for_pre_align_ = false;

  holdCurrentPosition(ctx);

  RCLCPP_WARN(
    logger_,
    "[TRAJECTORY] exit phase=%d index=%zu/%zu",
    static_cast<int>(phase_),
    index_,
    waypoints_.size());
}

void TrajectoryTask::onPause(Context& ctx)
{
  /*
   * Scheduler::interrupt() 会先调用当前任务 onPause()，
   * 然后把当前任务压入 paused_stack。
   */
  paused_for_pre_align_ = waiting_pre_align_;

  if (paused_for_pre_align_) {
    RCLCPP_WARN(
      logger_,
      "[TRAJECTORY] paused for RACK_PRE_ALIGN | "
      "rough_wp=%zu input=%zu "
      "actual=(%.3f %.3f %.3f)",
      pre_align_waypoint_index_,
      waypoints_[pre_align_waypoint_index_].input_index,
      ctx.cx(),
      ctx.cy(),
      ctx.cz());
  } else {
    RCLCPP_WARN(
      logger_,
      "[TRAJECTORY] paused by general interrupt | "
      "phase=%d wp=%zu",
      static_cast<int>(phase_),
      index_);
  }
}

void TrajectoryTask::onResume(Context& ctx)
{
  /*
   * 无论什么中断，恢复时都先让 cmd 从飞机当前实际位置开始。
   *
   * 注意：
   * 当前位置只作为平滑控制的起点，
   * 不会被添加到 waypoints_，
   * 也不会用于重新计算“下一个航点”。
   */
  cmd_x_ = static_cast<double>(ctx.cx());
  cmd_y_ = static_cast<double>(ctx.cy());
  cmd_z_ = static_cast<double>(ctx.cz());

  if (!paused_for_pre_align_) {
    RCLCPP_WARN(
      logger_,
      "[TRAJECTORY] resumed from general interrupt | "
      "continue original wp=%zu",
      index_);

    publishSetpoint(ctx);
    return;
  }

  if (pre_align_waypoint_index_ >= waypoints_.size()) {
    RCLCPP_ERROR(
      logger_,
      "[TRAJECTORY] invalid saved rough waypoint index=%zu size=%zu",
      pre_align_waypoint_index_,
      waypoints_.size());

    waiting_pre_align_ = false;
    paused_for_pre_align_ = false;
    ctx.front_pre_align_request = false;

    phase_ = Phase::FAILED;
    return;
  }

  const size_t completed_rough_wp =
    pre_align_waypoint_index_;

  const size_t completed_input_index =
    waypoints_[completed_rough_wp].input_index;

  /*
   * 核心逻辑：
   *
   * 1. 恢复原粗定位点的 index
   * 2. 调用一次 nextWaypoint()
   *
   * 例如：
   * 原轨迹 P0 P1 P2 P3
   * P1 触发前置矫正
   *
   * pre_align_waypoint_index_ = 1
   * 恢复时：
   *
   * index_ = 1
   * nextWaypoint()
   * index_ = 2
   *
   * 因此下一目标严格是原轨迹 P2。
   *
   * 不平移 P2/P3。
   * 不重新 buildWaypoints()。
   * 不使用当前实际位置生成新航点。
   */
  index_ = completed_rough_wp;

  waiting_pre_align_ = false;
  paused_for_pre_align_ = false;

  ctx.front_pre_align_request = false;

  hover_elapsed_s_ = 0.0;

  nextWaypoint();

  if (phase_ == Phase::FINISHED) {
    RCLCPP_WARN(
      logger_,
      "[TRAJECTORY] RACK_PRE_ALIGN finished "
      "at last waypoint | aligned=%d "
      "rough_wp=%zu input=%zu",
      ctx.vision_aligned ? 1 : 0,
      completed_rough_wp,
      completed_input_index);

    holdCurrentPosition(ctx);
    return;
  }

  const Waypoint& next_wp = waypoints_[index_];

  RCLCPP_WARN(
    logger_,
    "[TRAJECTORY] RACK_PRE_ALIGN finished | "
    "aligned=%d "
    "rough_wp=%zu input=%zu -> "
    "original_next_wp=%zu input=%zu "
    "current=(%.3f %.3f %.3f) "
    "target=(%.3f %.3f %.3f)",
    ctx.vision_aligned ? 1 : 0,
    completed_rough_wp,
    completed_input_index,
    index_,
    next_wp.input_index,
    ctx.cx(),
    ctx.cy(),
    ctx.cz(),
    next_wp.x,
    next_wp.y,
    next_wp.z);

  /*
   * 此时 cmd 是矫正结束后的当前位置，
   * 下一次 tick 会从该位置平滑推进到原始 next_wp。
   */
  publishSetpoint(ctx);
}

void TrajectoryTask::onCancel(Context& ctx)
{
  ctx.front_pre_align_request = false;

  waiting_pre_align_ = false;
  paused_for_pre_align_ = false;

  holdCurrentPosition(ctx);

  phase_ = Phase::FAILED;

  RCLCPP_WARN(
    logger_,
    "[TRAJECTORY] cancelled");
}

void TrajectoryTask::setWaypoints(
  const std::vector<InputWaypoint>& waypoints)
{
  cfg_.waypoints = waypoints;
}

void TrajectoryTask::buildWaypoints()
{
  waypoints_.clear();

  for (size_t i = 0; i < cfg_.waypoints.size(); ++i) {
    Waypoint resolved;

    if (!makeResolvedWaypoint(
        cfg_.waypoints[i],
        i,
        resolved))
    {
      RCLCPP_ERROR(
        logger_,
        "[TRAJECTORY] invalid input waypoint index=%zu",
        i);

      waypoints_.clear();
      phase_ = Phase::FAILED;
      return;
    }

    waypoints_.push_back(resolved);
  }

  if (cfg_.enable_tsp_sort &&
      waypoints_.size() > 1)
  {
    sortWaypointsNearestNeighbor();
  }
}


bool TrajectoryTask::makeResolvedWaypoint(
  const InputWaypoint& in,
  size_t input_index,
  Waypoint& out) const
{
  if (!std::isfinite(in.x) ||
      !std::isfinite(in.y) ||
      !std::isfinite(in.z))
  {
    return false;
  }

  if (in.use_yaw &&
      !std::isfinite(in.yaw))
  {
    return false;
  }

  switch (cfg_.coord_mode) {
    case CoordMode::ABSOLUTE_LOCAL:
      out.x = in.x;
      out.y = in.y;
      out.z = in.z;
      break;

    case CoordMode::RELATIVE_TO_ENTER:
      out.x = enter_x_ + in.x;
      out.y = enter_y_ + in.y;
      out.z = enter_z_ + in.z;
      break;

    default:
      return false;
  }

  out.hover_after = in.hover_after;

  if (in.hover_s >= 0.0) {
    out.hover_s = in.hover_s;
  } else {
    out.hover_s = cfg_.default_hover_s;
  }

  if (!std::isfinite(out.hover_s)) {
    return false;
  }

  out.hover_s = std::max(0.0, out.hover_s);

  out.use_yaw = in.use_yaw;
  out.yaw = in.yaw;

  out.pre_align_after = in.pre_align_after;

  out.input_index = input_index;

  return
    std::isfinite(out.x) &&
    std::isfinite(out.y) &&
    std::isfinite(out.z);
}
//tsp 最近邻排序
void TrajectoryTask::sortWaypointsNearestNeighbor()
{
  std::vector<Waypoint> remaining = waypoints_;
  std::vector<Waypoint> ordered;

  ordered.reserve(remaining.size());

  double current_x = enter_x_;
  double current_y = enter_y_;
  double current_z = enter_z_;

  while (!remaining.empty()) {
    size_t best_index = 0;
    double best_dist_sq =
      std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < remaining.size(); ++i) {
      const double dx = remaining[i].x - current_x;
      const double dy = remaining[i].y - current_y;
      const double dz = remaining[i].z - current_z;

      const double dist_sq =
        dx * dx +
        dy * dy +
        dz * dz;

      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_index = i;
      }
    }

    Waypoint selected = remaining[best_index];

    current_x = selected.x;
    current_y = selected.y;
    current_z = selected.z;

    ordered.push_back(selected);

    remaining.erase(
      remaining.begin() +
      static_cast<std::ptrdiff_t>(best_index));
  }

  waypoints_ = std::move(ordered);
}

void TrajectoryTask::moveCommandToward(
  const Waypoint& wp)
{
  const double dx = wp.x - cmd_x_;
  const double dy = wp.y - cmd_y_;
  const double dz = wp.z - cmd_z_;

  const double distance =
    std::sqrt(
      dx * dx +
      dy * dy +
      dz * dz);

  if (!std::isfinite(distance)) {
    return;
  }

  if (distance <= cfg_.max_step_m ||
      distance <= 1e-9)
  {
    cmd_x_ = wp.x;
    cmd_y_ = wp.y;
    cmd_z_ = wp.z;
    return;
  }

  const double scale =
    cfg_.max_step_m / distance;

  cmd_x_ += dx * scale;
  cmd_y_ += dy * scale;
  cmd_z_ += dz * scale;
}

bool TrajectoryTask::arrived(
  const Context& ctx,
  const Waypoint& wp) const
{
  const double dx =
    static_cast<double>(ctx.cx()) - wp.x;

  const double dy =
    static_cast<double>(ctx.cy()) - wp.y;

  const double dz =
    static_cast<double>(ctx.cz()) - wp.z;

  const double error_xy =
    std::hypot(dx, dy);

  const double error_z =
    std::fabs(dz);

  return
    error_xy <= cfg_.arrive_xy_m &&
    error_z <= cfg_.arrive_z_m;
}

void TrajectoryTask::publishSetpoint(
  Context& ctx)
{
  ctx.use_vel_ctrl = false;

  ctx.sp_x = static_cast<float>(cmd_x_);
  ctx.sp_y = static_cast<float>(cmd_y_);
  ctx.sp_z = static_cast<float>(cmd_z_);

  if (index_ < waypoints_.size()) {
    ctx.sp_yaw =
      static_cast<float>(
        waypointYaw(waypoints_[index_]));
  } else {
    ctx.sp_yaw =
      static_cast<float>(yaw_hold_);
  }

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  ctx.sp_ax = 0.0f;
  ctx.sp_ay = 0.0f;
  ctx.sp_az = 0.0f;
}
// 当前航点完成后的处理：
// 1. 需要前置矫正则发出中断请求
// 2. 否则进入原轨迹下一个航点
void TrajectoryTask::completeCurrentWaypoint(
  Context& ctx)
{
  if (index_ >= waypoints_.size()) {
    phase_ = Phase::FINISHED;
    return;
  }

  const Waypoint& wp = waypoints_[index_];

  if (wp.pre_align_after) {
    requestPreAlign(ctx);
    return;
  }

  nextWaypoint();
}
//触发中断并且保持当前位置
void TrajectoryTask::requestPreAlign(
  Context& ctx)
{
  if (index_ >= waypoints_.size()) {
    phase_ = Phase::FAILED;
    return;
  }

  /*
   * 保存触发中断的原轨迹粗定位点编号。
   *
   * 即使视觉矫正把飞机移动到了其他位置，
   * 这个编号也不会改变。
   */
  pre_align_waypoint_index_ = index_;

  waiting_pre_align_ = true;
  paused_for_pre_align_ = false;

  // 清除上一次视觉矫正结果
  ctx.vision_aligned = false;

  ctx.front_pre_align_request = true;

  /*
   * 在 Scheduler 真正切换到辅助任务之前，
   * 固定保持当前实际位置。
   */
  interrupt_hold_x_ =
    static_cast<double>(ctx.cx());

  interrupt_hold_y_ =
    static_cast<double>(ctx.cy());

  interrupt_hold_z_ =
    static_cast<double>(ctx.cz());

  cmd_x_ = interrupt_hold_x_;
  cmd_y_ = interrupt_hold_y_;
  cmd_z_ = interrupt_hold_z_;

  publishSetpoint(ctx);

  const Waypoint& wp = waypoints_[index_];

  RCLCPP_WARN(
    logger_,
    "[TRAJECTORY] request RACK_PRE_ALIGN | "
    "rough_wp=%zu input=%zu "
    "rough_target=(%.3f %.3f %.3f) "
    "hold=(%.3f %.3f %.3f)",
    index_,
    wp.input_index,
    wp.x,
    wp.y,
    wp.z,
    interrupt_hold_x_,
    interrupt_hold_y_,
    interrupt_hold_z_);
}

void TrajectoryTask::nextWaypoint()
{
  hover_elapsed_s_ = 0.0;

  if (waypoints_.empty() ||
      index_ + 1 >= waypoints_.size())
  {
    index_ = waypoints_.size();
    phase_ = Phase::FINISHED;
    return;
  }

  ++index_;
  phase_ = Phase::MOVING;
}

double TrajectoryTask::waypointYaw(
  const Waypoint& wp) const
{
  return wp.use_yaw ?
    wp.yaw :
    yaw_hold_;
}

void TrajectoryTask::holdCurrentPosition(
  Context& ctx)
{
  if (ctx.pos_valid()) {
    cmd_x_ = static_cast<double>(ctx.cx());
    cmd_y_ = static_cast<double>(ctx.cy());
    cmd_z_ = static_cast<double>(ctx.cz());
  }

  ctx.use_vel_ctrl = false;

  ctx.sp_x = static_cast<float>(cmd_x_);
  ctx.sp_y = static_cast<float>(cmd_y_);
  ctx.sp_z = static_cast<float>(cmd_z_);

  ctx.sp_yaw = static_cast<float>(yaw_hold_);

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  ctx.sp_ax = 0.0f;
  ctx.sp_ay = 0.0f;
  ctx.sp_az = 0.0f;
}
