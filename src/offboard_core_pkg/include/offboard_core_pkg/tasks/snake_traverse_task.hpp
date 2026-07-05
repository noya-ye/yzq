#pragma once

#include "rclcpp/rclcpp.hpp"
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/math_tool.hpp"

namespace offboard_core_pkg
{

class SnakeTraverseTask : public ITask
{
public:
  SnakeTraverseTask(rclcpp::Logger lg,
                    int rows,
                    int cols,
                    float cell_size,
                    float origin_dx,
                    float origin_dy,
                    float xy_tol = 0.2f,
                    float z_tol = 0.2f,
                    float hover_time = 2.0f);

  std::string name() const override { return "SNAKE_TRAVERSE"; }

  void onEnter(Context& ctx) override;
  Status tick(Context& ctx, double dt) override;

private:

  // ================= 主状态 =================
  enum class SnakeState
{
  MOVE_TO_ROW_START,
  MOVE_ROW,
  HOVER_ROW,
  FINISHED
};

  // ================= 中断状态 =================
  enum class VisionState
  {
    IDLE,
    INTERRUPT_HOVER,
    ALIGN,          // 🔥替换 MOVE_TO_TARGET
    IDENTIFY,
    RETURN_BACK
  };

  rclcpp::Logger lg_;

  int rows_, cols_;
  float cell_size_;
  float origin_dx_, origin_dy_;

  float xy_tol_, z_tol_;
  float hover_time_;

  SnakeState snake_state_{SnakeState::MOVE_ROW};
  VisionState vision_state_{VisionState::IDLE};

  int current_row_{0};

  float row_start_x_, row_start_y_;
  float row_end_x_, row_end_y_;

  float hover_timer_{0.0f};
  float hover_x_{0.0f};
    float hover_y_{0.0f};
    float hover_z_{0.0f};

  // ===== 中断恢复 =====
  bool in_interrupt_{false};

  float resume_x_, resume_y_, resume_z_, resume_yaw_;

  // ================= 工具 =================
  void compute_row(Context& ctx);
  bool reached(Context& ctx, float x, float y);
};

} // namespace offboard_core_pkg