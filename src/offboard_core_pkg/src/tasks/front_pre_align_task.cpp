#include "offboard_core_pkg/tasks/front_pre_align_task.hpp"

#include <algorithm>
#include <cmath>

namespace offboard_core_pkg
{

FrontPreAlignTask::FrontPreAlignTask(
  rclcpp::Logger logger,
  double timeout_s,
  double align_tol_m,
  int stable_required,
  double k_img_to_meter,
  double max_step_m,
  double score_thresh,
  double img_to_body_y_sign,
  double img_to_body_z_sign,
  int expected_type)
: logger_(logger),
  timeout_s_(timeout_s),
  align_tol_m_(align_tol_m),
  stable_required_(stable_required),
  k_img_to_meter_(k_img_to_meter),
  max_step_m_(max_step_m),
  score_thresh_(score_thresh),
  img_to_body_y_sign_(img_to_body_y_sign),
  img_to_body_z_sign_(img_to_body_z_sign),
  expected_type_(expected_type)
{
}

std::string FrontPreAlignTask::name() const
{
  return "FRONT_PRE_ALIGN";
}

void FrontPreAlignTask::onEnter(Context& ctx)
{
  elapsed_ = 0.0;
  print_elapsed_ = 0.0;
  frame_age_s_ = 999.0;

  stable_count_ = 0;

  enter_failed_ = false;
  ever_seen_ = false;

  last_frame_stamp_us_ = 0;

  lateral_cmd_m_ = 0.0f;
  vertical_cmd_m_ = 0.0f;

  if (!ctx.pos_valid() ||
      !ctx.has_attitude ||
      !std::isfinite(ctx.yaw))
  {
    RCLCPP_ERROR(
      logger_,
      "[FrontPreAlignTask] enter failed: "
      "pos_valid=%d has_attitude=%d yaw=%.3f",
      ctx.pos_valid() ? 1 : 0,
      ctx.has_attitude ? 1 : 0,
      ctx.yaw);

    enter_failed_ = true;
    return;
  }

  // 保存进入任务时的货架观察位姿
  origin_x_ = ctx.cx();
  origin_y_ = ctx.cy();
  origin_z_ = ctx.cz();
  hold_yaw_ = ctx.yaw;

  // 清除旧视觉数据
  ctx.vision_offset = VisionOffset{};
  ctx.vision_last_update_us = 0;

  ctx.vision_aligned = false;
  ctx.vision_target_locked = false;
  ctx.vision_searching = true;

  // 开启前视、关闭下视
  ctx.vision_down_enable = false;
  ctx.vision_front_enable = true;
  ctx.vision_enable = true;

  update_hold_setpoint(ctx);

  RCLCPP_WARN(
    logger_,
    "[FrontPreAlignTask] enter "
    "origin=(%.3f %.3f %.3f) yaw=%.3f "
    "tol=%.3f stable=%d k=%.3f step=%.3f "
    "score=%.1f expected_type=%d",
    origin_x_,
    origin_y_,
    origin_z_,
    hold_yaw_,
    align_tol_m_,
    stable_required_,
    k_img_to_meter_,
    max_step_m_,
    score_thresh_,
    expected_type_);
}

ITask::Status FrontPreAlignTask::tick(Context& ctx, double dt)
{
  /*
   * Scheduler 在 FAILURE 时会直接结束整个任务链。
   * 因此本任务异常或超时时返回 SUCCESS，
   * 但通过 ctx.vision_aligned=false 表示没有完成对准。
   */
  if (enter_failed_) {
    finish(ctx, false);
    return Status::SUCCESS;
  }

  const double safe_dt = std::clamp(dt, 0.0, 0.20);

  elapsed_ += safe_dt;
  print_elapsed_ += safe_dt;

  // 只把 stamp_us 改变视为真正的新视觉帧
  const uint64_t stamp_us = ctx.vision_offset.stamp_us;

  const bool new_frame =
    stamp_us != 0 &&
    stamp_us != last_frame_stamp_us_;

  if (new_frame) {
    last_frame_stamp_us_ = stamp_us;
    frame_age_s_ = 0.0;
  } else {
    frame_age_s_ += safe_dt;
  }

  // 每一拍都维持前视门控
  ctx.vision_down_enable = false;
  ctx.vision_front_enable = true;
  ctx.vision_enable = true;

  ctx.vision_searching = true;
  ctx.use_vel_ctrl = false;
  ctx.sp_yaw = hold_yaw_;

  // 总超时
  if (elapsed_ >= timeout_s_) {
    RCLCPP_ERROR(
      logger_,
      "[FrontPreAlignTask] timeout %.2fs "
      "ever_seen=%d stable=%d/%d",
      elapsed_,
      ever_seen_ ? 1 : 0,
      stable_count_,
      stable_required_);

    finish(ctx, false);
    return Status::SUCCESS;
  }

  // 当前没有可靠目标
  if (!target_valid(ctx)) {
    stable_count_ = 0;
    ctx.vision_target_locked = false;

    update_hold_setpoint(ctx);

    if (print_elapsed_ >= 0.5) {
      print_elapsed_ = 0.0;

      RCLCPP_WARN(
        logger_,
        "[FrontPreAlignTask] waiting target | "
        "frame_age=%.3f type=%d score=%.1f "
        "stamp=%llu elapsed=%.2f",
        frame_age_s_,
        ctx.vision_offset.type,
        ctx.vision_offset.score,
        static_cast<unsigned long long>(
          ctx.vision_offset.stamp_us),
        elapsed_);
    }

    return Status::RUNNING;
  }

  ever_seen_ = true;
  ctx.vision_target_locked = true;

  /*
   * 防止一个视觉帧被 50Hz 控制循环重复计算。
   * 只有 stamp_us 发生变化时才进行一次纠偏。
   */
  if (!new_frame) {
    update_hold_setpoint(ctx);
    return Status::RUNNING;
  }

  const float dx = ctx.vision_offset.dx;
  const float dy = ctx.vision_offset.dy;

  // 将二维图像误差估算为实际误差
  const float err_m =
    static_cast<float>(k_img_to_meter_) *
    std::hypot(dx, dy);

  const float vxy =
    std::hypot(
      ctx.local_pos.vx,
      ctx.local_pos.vy);

  const float vz =
    std::fabs(ctx.local_pos.vz);

  const bool low_speed =
    vxy <= static_cast<float>(stable_vxy_mps_) &&
    vz <= static_cast<float>(stable_vz_mps_);

  // 对准且飞机已经稳定
  if (err_m <= static_cast<float>(align_tol_m_) &&
      low_speed)
  {
    ++stable_count_;

    if (stable_count_ >= stable_required_) {
      RCLCPP_WARN(
        logger_,
        "[FrontPreAlignTask] aligned success | "
        "err=%.3fm stable=%d/%d "
        "pos=(%.3f %.3f %.3f)",
        err_m,
        stable_count_,
        stable_required_,
        ctx.cx(),
        ctx.cy(),
        ctx.cz());

      finish(ctx, true);
      return Status::SUCCESS;
    }
  } else {
    stable_count_ = 0;

    /*
     * 前视相机映射：
     *
     * dx > 0：
     * 目标在图像右边，向机体系右侧修正。
     *
     * dy > 0：
     * 目标在图像下方，NED z 增大，即降低高度。
     */
    const float lateral_error_m =
      static_cast<float>(
        img_to_body_y_sign_ *
        k_img_to_meter_ *
        static_cast<double>(dx));

    const float vertical_error_m =
      static_cast<float>(
        img_to_body_z_sign_ *
        k_img_to_meter_ *
        static_cast<double>(dy));

    const float lateral_step =
      std::clamp(
        lateral_error_m,
        -static_cast<float>(max_step_m_),
        static_cast<float>(max_step_m_));

    const float vertical_step =
      std::clamp(
        vertical_error_m,
        -static_cast<float>(max_step_m_),
        static_cast<float>(max_step_m_));

    /*
     * 机体系右方向在世界坐标中的方向：
     *
     * body_y_world =
     *   (-sin(yaw), cos(yaw))
     */
    const float right_x = -std::sin(hold_yaw_);
    const float right_y =  std::cos(hold_yaw_);

    // 当前实际横向偏移
    const float actual_lateral_m =
      (ctx.cx() - origin_x_) * right_x +
      (ctx.cy() - origin_y_) * right_y;

    // 横向控制量
    lateral_cmd_m_ =
      std::clamp(
        actual_lateral_m + lateral_step,
        -static_cast<float>(max_lateral_move_m_),
        static_cast<float>(max_lateral_move_m_));

    // 高度控制量
    vertical_cmd_m_ =
      std::clamp(
        (ctx.cz() - origin_z_) + vertical_step,
        -static_cast<float>(max_vertical_move_m_),
        static_cast<float>(max_vertical_move_m_));
  }

  update_hold_setpoint(ctx);

  if (print_elapsed_ >= 0.5) {
    print_elapsed_ = 0.0;

    RCLCPP_INFO(
      logger_,
      "[FrontPreAlignTask] "
      "type=%d dx=%.3f dy=%.3f err=%.3fm "
      "cmd_lat=%.3f cmd_z=%.3f "
      "speed=(%.3f %.3f) stable=%d/%d",
      ctx.vision_offset.type,
      dx,
      dy,
      err_m,
      lateral_cmd_m_,
      vertical_cmd_m_,
      vxy,
      vz,
      stable_count_,
      stable_required_);
  }

  return Status::RUNNING;
}

void FrontPreAlignTask::onExit(Context& ctx)
{
  ctx.vision_front_enable = false;
  ctx.vision_enable = false;

  ctx.vision_searching = false;
  ctx.vision_target_locked = false;
}

bool FrontPreAlignTask::target_valid(
  const Context& ctx) const
{
  const VisionOffset& v = ctx.vision_offset;

  // 超过一定时间没有收到新帧
  if (frame_age_s_ > frame_timeout_s_) {
    return false;
  }

  if (v.stamp_us == 0) {
    return false;
  }

  if (v.type == 0) {
    return false;
  }

  // expected_type=-1 时接受所有非零类型
  if (expected_type_ > 0 &&
      v.type != expected_type_)
  {
    return false;
  }

  if (v.score < static_cast<float>(score_thresh_)) {
    return false;
  }

  return
    std::isfinite(v.dx) &&
    std::isfinite(v.dy);
}

void FrontPreAlignTask::update_hold_setpoint(
  Context& ctx)
{
  const float right_x = -std::sin(hold_yaw_);
  const float right_y =  std::cos(hold_yaw_);

  ctx.use_vel_ctrl = false;

  /*
   * 始终使用进入任务时的位置作为货架法向基准，
   * 因此任务不会主动接近或远离货架。
   */
  ctx.sp_x =
    origin_x_ +
    right_x * lateral_cmd_m_;

  ctx.sp_y =
    origin_y_ +
    right_y * lateral_cmd_m_;

  ctx.sp_z =
    origin_z_ +
    vertical_cmd_m_;

  ctx.sp_yaw = hold_yaw_;

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  ctx.sp_ax = 0.0f;
  ctx.sp_ay = 0.0f;
  ctx.sp_az = 0.0f;
}

void FrontPreAlignTask::finish(
  Context& ctx,
  bool aligned)
{
  ctx.vision_front_enable = false;
  ctx.vision_down_enable = false;
  ctx.vision_enable = false;

  ctx.vision_searching = false;
  ctx.vision_target_locked = false;
  ctx.vision_aligned = aligned;

  // 完成时保持当前实际位置
  ctx.use_vel_ctrl = false;

  ctx.sp_x = ctx.cx();
  ctx.sp_y = ctx.cy();
  ctx.sp_z = ctx.cz();
  ctx.sp_yaw = hold_yaw_;

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  ctx.sp_ax = 0.0f;
  ctx.sp_ay = 0.0f;
  ctx.sp_az = 0.0f;
}

}  // namespace offboard_core_pkg
