#pragma once

#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/grid.hpp"
#include "offboard_core_pkg/tasks/grid_goto_dwell_task.hpp"

class GridGotoDwellTask : public ITask {
public:
  GridGotoDwellTask(rclcpp::Logger lg,
                    Grid& grid,
                    double wp_xy_tol,
                    double wp_z_tol,
                    double v_xy_tol,
                    double v_z_tol,
                    int stable_required,
                    double dwell_s);

  std::string name() const override;

  void onEnter(Context& ctx) override;
  ITask::Status tick(Context& ctx, double dt);
protected:
  // ===== 可复用基础能力 =====
  void ensure_grid_ready(Context& ctx);
  void set_hover_sp(Context& ctx, float x, float y);
  bool reached_and_stable(Context& ctx, float tx, float ty);

  // ===== 给子类覆写的钩子 =====
  virtual void on_task_enter(Context& ctx) {}
  virtual void on_before_goto(Context& ctx, int r, int c, float& tx, float& ty) {}
  virtual void on_cell_enter(Context& ctx, int r, int c) {}
  virtual void on_cell_dwell(Context& ctx, int r, int c, double dwell_t, double dt) {}
  virtual void on_cell_leave(Context& ctx, int r, int c) {}
  virtual void on_task_finish(Context& ctx) {}
  virtual bool should_finish_early(Context& ctx) { return false; }

  // DWELL 阶段是否允许离开当前点。
  // base_timeout 表示原始 dwell_s_ 是否已经到时。
  // 默认逻辑：base_timeout=true 就离开。
  // 子类可重写，比如 TSP 要求视觉误差足够小才去下一个点。
  virtual bool should_leave_cell(Context& ctx, double dwell_t, bool base_timeout);

protected:
  rclcpp::Logger lg_;
  Grid& grid_;

  double wp_xy_tol_{0.25};
  double wp_z_tol_{0.25};
  double v_xy_tol_{0.2};
  double v_z_tol_{0.2};
  int stable_required_{12};
  double dwell_s_{5.0};

  int stable_counter_{0};
  bool in_dwell_{false};
  double dwell_t_{0.0};
};