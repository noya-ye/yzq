#include "offboard_core_pkg/tasks/snake_traverse_task.hpp"

namespace offboard_core_pkg
{

SnakeTraverseTask::SnakeTraverseTask(rclcpp::Logger lg,
                                     int rows,
                                     int cols,
                                     float cell_size,
                                     float origin_dx,
                                     float origin_dy,
                                     float xy_tol,
                                     float z_tol,
                                     float hover_time)
: lg_(lg),
  rows_(rows),
  cols_(cols),
  cell_size_(cell_size),
  origin_dx_(origin_dx),
  origin_dy_(origin_dy),
  xy_tol_(xy_tol),
  z_tol_(z_tol),
  hover_time_(hover_time)
{}

void SnakeTraverseTask::onEnter(Context& ctx)
{
  current_row_ = 0;

  // 关键修正：必须先到当前列起点，不能一开始就直接飞终点
  snake_state_ = SnakeState::MOVE_TO_ROW_START;

  vision_state_ = VisionState::IDLE;
  in_interrupt_ = false;
  hover_timer_ = 0.0f;

  hover_x_ = hover_y_ = hover_z_ = 0.0f;
  resume_x_ = resume_y_ = resume_z_ = resume_yaw_ = 0.0f;

  RCLCPP_INFO(lg_, "[Snake] Start");
}

bool SnakeTraverseTask::reached(Context& ctx, float x, float y)
{
  return math_tool::reached_xy(ctx.cx(), ctx.cy(), x, y, xy_tol_);
}

// 先上后右：
// 第0列：下 -> 上
// 第1列：上 -> 下
// 第2列：下 -> 上
void SnakeTraverseTask::compute_row(Context& ctx)
{
  float base_x = ctx.home_x + origin_dx_;
  float base_y = ctx.home_y + origin_dy_;

  // current_row_ 现在逻辑上更像“第几列”
  float x = base_x + current_row_ * cell_size_;

  if (current_row_ % 2 == 0)
  {
    // 偶数列：下 -> 上
    row_start_x_ = x;
    row_start_y_ = base_y;

    row_end_x_   = x;
    row_end_y_   = base_y + (cols_ - 1) * cell_size_;
  }
  else
  {
    // 奇数列：上 -> 下
    row_start_x_ = x;
    row_start_y_ = base_y + (cols_ - 1) * cell_size_;

    row_end_x_   = x;
    row_end_y_   = base_y;
  }
}

ITask::Status SnakeTraverseTask::tick(Context& ctx, double dt)
{
  if (!ctx.pos_valid()) {
    return Status::RUNNING;
  }

  ctx.sp_yaw = ctx.home_yaw;

  // ================== 中断触发 ==================
  if (ctx.vision_detected && !in_interrupt_)
  {
    in_interrupt_ = true;

    resume_x_ = ctx.cx();
    resume_y_ = ctx.cy();
    resume_z_ = ctx.cz();
    resume_yaw_ = ctx.yaw;

    vision_state_ = VisionState::INTERRUPT_HOVER;

    RCLCPP_WARN(lg_, "[Snake] INTERRUPT TRIGGERED");
    return Status::RUNNING;
  }

  // ================== 中断流程 ==================
  if (in_interrupt_)
  {
    switch (vision_state_)
    {
      case VisionState::INTERRUPT_HOVER:
      {
        ctx.use_vel_ctrl = false;
        ctx.sp_x = ctx.cx();
        ctx.sp_y = ctx.cy();
        ctx.sp_z = ctx.cz();
        ctx.sp_yaw = ctx.home_yaw;

        if (ctx.is_low_speed())
        {
          vision_state_ = VisionState::ALIGN;
        }
        break;
      }

      case VisionState::ALIGN:
      {
        auto& off = ctx.vision_offset;

        ctx.use_vel_ctrl = true;

        float k = 0.6f;

        // 注意：这里你原本的映射我先保持不动
        ctx.sp_vx = -k * off.dy;
        ctx.sp_vy = -k * off.dx;
        ctx.sp_vz = 0.0f;

        ctx.sp_vx = math_tool::clamp(ctx.sp_vx, -0.5f, 0.5f);
        ctx.sp_vy = math_tool::clamp(ctx.sp_vy, -0.5f, 0.5f);

        if (std::fabs(off.dx) < 0.03f && std::fabs(off.dy) < 0.03f)
        {
          ctx.use_vel_ctrl = false;
          ctx.sp_x = ctx.cx();
          ctx.sp_y = ctx.cy();
          ctx.sp_z = ctx.cz();
          ctx.sp_yaw = ctx.home_yaw;

          vision_state_ = VisionState::IDENTIFY;
          RCLCPP_INFO(lg_, "[Vision] ALIGN DONE");
        }
        break;
      }

      case VisionState::IDENTIFY:
      {
        ctx.use_vel_ctrl = false;
        ctx.sp_x = ctx.cx();
        ctx.sp_y = ctx.cy();
        ctx.sp_z = ctx.cz();
        ctx.sp_yaw = ctx.home_yaw;

        if (ctx.vision_done)
        {
          RCLCPP_INFO(lg_,
                      "[TARGET] pos=(%.2f, %.2f) type=%d",
                      ctx.cx(), ctx.cy(), (int)ctx.vision.type);

          vision_state_ = VisionState::RETURN_BACK;
        }
        break;
      }

      case VisionState::RETURN_BACK:
      {
        ctx.use_vel_ctrl = false;
        ctx.sp_x = resume_x_;
        ctx.sp_y = resume_y_;
        ctx.sp_z = resume_z_;
        ctx.sp_yaw = ctx.home_yaw;

        if (reached(ctx, resume_x_, resume_y_))
        {
          in_interrupt_ = false;
          ctx.vision_detected = false;
          ctx.vision_done = false;
          vision_state_ = VisionState::IDLE;

          RCLCPP_INFO(lg_, "[Snake] Resume");
        }
        break;
      }

      default:
        break;
    }

    return Status::RUNNING;
  }

  // ================== 主蛇形 ==================
  if (snake_state_ == SnakeState::FINISHED) {
    return Status::SUCCESS;
  }

  compute_row(ctx);

  switch (snake_state_)
  {
    case SnakeState::MOVE_TO_ROW_START:
    {
      // 先到当前列起点
      ctx.use_vel_ctrl = false;
      ctx.sp_x = row_start_x_;
      ctx.sp_y = row_start_y_;
      ctx.sp_z = ctx.takeoff_z;
      ctx.sp_yaw = ctx.home_yaw;

      if (reached(ctx, row_start_x_, row_start_y_))
      {
        snake_state_ = SnakeState::MOVE_ROW;
        RCLCPP_INFO(lg_, "[Snake] Reached row start %d", current_row_);
      }
      break;
    }

    case SnakeState::MOVE_ROW:
    {
      // 再沿当前列飞到终点
      ctx.use_vel_ctrl = false;
      ctx.sp_x = row_end_x_;
      ctx.sp_y = row_end_y_;
      ctx.sp_z = ctx.takeoff_z;
      ctx.sp_yaw = ctx.home_yaw;

      if (reached(ctx, row_end_x_, row_end_y_))
      {
        snake_state_ = SnakeState::HOVER_ROW;
        hover_timer_ = 0.0f;

        // 锁定悬停点，不要每拍都用实时位置
        hover_x_ = ctx.cx();
        hover_y_ = ctx.cy();
        hover_z_ = ctx.cz();

        RCLCPP_INFO(lg_, "[Snake] Row %d finished", current_row_);
      }
      break;
    }

    case SnakeState::HOVER_ROW:
    {
      ctx.use_vel_ctrl = false;
      ctx.sp_x = hover_x_;
      ctx.sp_y = hover_y_;
      ctx.sp_z = hover_z_;
      ctx.sp_yaw = ctx.home_yaw;

      hover_timer_ += static_cast<float>(dt);

      if (hover_timer_ > hover_time_)
      {
        current_row_++;

        if (current_row_ >= rows_)
        {
          snake_state_ = SnakeState::FINISHED;
          RCLCPP_INFO(lg_, "[Snake] DONE");
        }
        else
        {
          snake_state_ = SnakeState::MOVE_TO_ROW_START;
        }
      }
      break;
    }

    case SnakeState::FINISHED:
    default:
      break;
  }

  return Status::RUNNING;
}

} // namespace offboard_core_pkg