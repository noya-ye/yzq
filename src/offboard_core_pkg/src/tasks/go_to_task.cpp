#include "offboard_core_pkg/tasks/go_to_task.hpp"

GoToTask::GoToTask(
  rclcpp::Logger lg,
  double xy_tol,
  double z_tol,
  int stable_required)
: lg_(lg),
  xy_tol_(xy_tol),
  z_tol_(z_tol),
  stable_required_(stable_required)
{}

std::string GoToTask::name() const
{
  return "GO_TO";
}

void GoToTask::onEnter(Context& ctx)
{
  stable_count_ = 0;
  has_target_ = false;

  if (ctx.detected_targets.empty()) {
    RCLCPP_WARN(lg_, "[GoToTask] ctx.detected_targets is empty, skip GO_TO");
    return;
  }

  // 取 detected_targets 最后一个点
  // 也就是地面站 GOTO 后追加进去的点
  const auto& p = ctx.detected_targets.back();

  // 地面站坐标定义：
  // 起飞点为原点，x 向前，y 向左
  //
  // PX4 local 坐标：
  // home_x/home_y 是起飞点在 local frame 下的位置
  target_x_ = ctx.home_x + p.x;
  target_y_ = ctx.home_y + p.y;

  // 高度固定为起飞高度
  // PX4 NED 坐标中，向上是负 z
  target_z_ = ctx.takeoff_z;

  has_target_ = true;

  RCLCPP_WARN(
    lg_,
    "[GoToTask] target selected from detected_targets.back(): rel=(%.2f, %.2f, %.2f), local=(%.2f, %.2f, %.2f)",
    p.x, p.y, p.z,
    target_x_, target_y_, target_z_);
}

ITask::Status GoToTask::tick(Context& ctx, double)
{
  if (!has_target_) {
    RCLCPP_WARN_THROTTLE(
      lg_,
      *rclcpp::Clock::make_shared(),
      1000,
      "[GoToTask] no target, return SUCCESS");

    return Status::SUCCESS;
  }

  // 使用位置控制
  ctx.use_vel_ctrl = false;

  ctx.sp_x = target_x_;
  ctx.sp_y = target_y_;
  ctx.sp_z = target_z_;
  ctx.sp_yaw = ctx.home_yaw;

  if (!ctx.pos_valid()) {
    stable_count_ = 0;

    RCLCPP_WARN_THROTTLE(
      lg_,
      *rclcpp::Clock::make_shared(),
      1000,
      "[GoToTask] waiting for valid local position");

    return Status::RUNNING;
  }

  const double dx = static_cast<double>(ctx.local_pos.x) - target_x_;
  const double dy = static_cast<double>(ctx.local_pos.y) - target_y_;
  const double dz = static_cast<double>(ctx.local_pos.z) - target_z_;

  const double xy_err = std::sqrt(dx * dx + dy * dy);
  const double z_err = std::fabs(dz);

  if (xy_err < xy_tol_ && z_err < z_tol_) {
    stable_count_++;
  } else {
    stable_count_ = 0;
  }

  RCLCPP_INFO_THROTTLE(
    lg_,
    *rclcpp::Clock::make_shared(),
    500,
    "[GoToTask] pos=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f) err_xy=%.2f err_z=%.2f stable=%d/%d",
    ctx.local_pos.x,
    ctx.local_pos.y,
    ctx.local_pos.z,
    target_x_,
    target_y_,
    target_z_,
    xy_err,
    z_err,
    stable_count_,
    stable_required_);

  if (stable_count_ >= stable_required_) {
    RCLCPP_INFO(lg_, "[GoToTask] reached target");
    return Status::SUCCESS;
  }

  return Status::RUNNING;
}
