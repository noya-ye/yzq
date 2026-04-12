#include "offboard_core_pkg/tasks/grid_goto_dwell_task.hpp"

#include <cmath>
#include "offboard_core_pkg/context.hpp"
/*
【快速检索】
on_task_enter()         任务开始时初始化
on_before_goto()        每次飞向格子前调用
on_cell_enter()         到达格子、刚进入悬停时调用
on_cell_dwell()         悬停期间每个 tick 调用
on_cell_leave()         悬停结束、离开格子前调用
on_task_finish()        任务结束时调用
should_finish_early()   判断是否提前结束

set_hover_sp()          设置位置悬停 setpoint
reached_and_stable()    判断是否到达并稳定
ensure_grid_ready()     确保 grid 已构建
*/

/*
======================================================================
                 GridGotoDwellTask 接口检索表 / 用法说明
======================================================================

【一、这个类是做什么的？】
----------------------------------------------------------------------
GridGotoDwellTask 是“网格遍历骨架任务”。

它只负责通用流程：
1. 按 Grid::waypoints 依次前往每个格子
2. 判断是否“到达且稳定”
3. 到达后进入悬停（DWELL）
4. 悬停结束后自动切到下一个格子
5. 全部遍历结束后返回 SUCCESS

它不负责具体比赛规则。
例如：
- 识别数字
- 识别颜色/图形
- 蓝色矩形抬升
- 红色圆提前返航
- 黄色六边形触发扫描
这些都应由子类重写钩子来实现。


【二、推荐使用方式】
----------------------------------------------------------------------
推荐写法：继承 GridGotoDwellTask，重写钩子函数

示例：
class ContestGridTask : public GridGotoDwellTask {
protected:
  void on_task_enter(Context& ctx) override;
  void on_cell_enter(Context& ctx, int r, int c) override;
  void on_cell_dwell(Context& ctx, int r, int c, double dwell_t, double dt) override;
  void on_cell_leave(Context& ctx, int r, int c) override;
  bool should_finish_early(Context& ctx) override;
};

然后在 build_scheduler() 中加入：
sched_.add(std::make_unique<ContestGridTask>(...));


【三、主流程 tick 执行顺序】
----------------------------------------------------------------------
onEnter()
  -> ensure_grid_ready()
  -> on_task_enter()

tick()
  -> update_current_cell()
  -> should_finish_early()
  -> 检查 wp_idx 是否结束
  -> on_before_goto()
  -> set_hover_sp()

  若当前处于 GOTO 阶段：
      reached_and_stable()
      若成立：
          on_cell_enter()
          进入 DWELL

  若当前处于 DWELL 阶段：
      on_cell_dwell()
      若 dwell 时间到：
          on_cell_leave()
          切换到下一个 waypoint

结束时：
  -> on_task_finish()


【四、构造函数参数说明】
----------------------------------------------------------------------
GridGotoDwellTask(
    rclcpp::Logger lg,
    Grid& grid,
    double wp_xy_tol,
    double wp_z_tol,
    double v_xy_tol,
    double v_z_tol,
    int stable_required,
    double dwell_s)

参数含义：

lg
  日志对象，用于 RCLCPP_INFO / WARN 输出

grid
  网格对象，内部提供：
  - waypoints
  - build(ctx)
  - update_current_cell(ctx)
  - wp_idx

wp_xy_tol
  到达目标点时的 XY 位置容差（米）

wp_z_tol
  到达目标点时的 Z 高度容差（米）

v_xy_tol
  判定“稳定”时的 XY 速度阈值（m/s）

v_z_tol
  判定“稳定”时的 Z 速度阈值（m/s）

stable_required
  连续满足“位置+速度稳定”多少次才算真正到达

dwell_s
  每个格子悬停时间（秒）


【五、基础接口说明（骨架内部会调用）】
----------------------------------------------------------------------

1) ensure_grid_ready(Context& ctx)
----------------------------------
作用：
  确保 grid 已经构建好。
  如果 grid_.waypoints 为空，则调用 grid_.build(ctx)。

典型用途：
  一般不需要子类主动调用，onEnter() 内部会自动调用。


2) set_hover_sp(Context& ctx, float x, float y)
-----------------------------------------------
作用：
  设置悬停位置控制 setpoint：
  - sp_x = x
  - sp_y = y
  - sp_z = ctx.takeoff_z
  - sp_yaw = ctx.home_yaw
  - use_vel_ctrl = false

典型用途：
  让无人机飞向指定网格中心点并保持当前任务高度。

说明：
  如果子类需要额外 yaw 修正，可以：
  - 在 on_before_goto() 或 on_cell_dwell() 中修改 ctx.sp_yaw
  - 或重写自己的辅助函数


3) reached_and_stable(Context& ctx, float tx, float ty)
-------------------------------------------------------
作用：
  判断当前是否已经到达目标点并稳定。

判定条件：
  - XY 距离小于 wp_xy_tol
  - Z 距离小于 wp_z_tol
  - XY 速度小于 v_xy_tol
  - Z 速度小于 v_z_tol
  - 连续 stable_required 次满足条件

返回值：
  true  -> 认为已经稳定到达
  false -> 还未稳定到达

典型用途：
  一般由骨架内部调用，子类通常不用直接调用。


【六、可重写钩子（最重要）】
----------------------------------------------------------------------

1) on_task_enter(Context& ctx)
------------------------------
调用时机：
  整个网格遍历任务刚开始时，在 onEnter() 的最后调用一次。

适合做什么：
  - 初始化任务内状态
  - 清零标志位
  - 开启/关闭视觉
  - 初始化存储数组
  - 清空 early_finish 标志

示例：
  void on_task_enter(Context& ctx) override {
    ctx.vision_enable = false;
    early_finish_ = false;
    ensure_storage_size(ctx);
  }


2) on_before_goto(Context& ctx, int r, int c, float tx, float ty)
------------------------------------------------------------------
调用时机：
  每次 tick 中，在 set_hover_sp() 之前调用。

参数：
  r, c  : 当前识别到的格子索引
  tx, ty: 当前目标 waypoint 坐标

适合做什么：
  - 在飞往目标前调整 setpoint
  - 添加 yaw 修正
  - 做日志输出
  - 做某些飞行前预处理

注意：
  这个钩子会频繁调用，每个 tick 都会进。


3) on_cell_enter(Context& ctx, int r, int c)
--------------------------------------------
调用时机：
  到达某个格子并稳定后，刚进入 DWELL 时调用一次。

适合做什么：
  - 重置当前格子的缓存
  - 清空数字候选
  - 开启视觉识别
  - 打印“到达某格子”的日志

示例：
  void on_cell_enter(Context& ctx, int r, int c) override {
    ctx.vision_enable = true;
    reset_candidate_for_cell(ctx, r, c);
  }


4) on_cell_dwell(Context& ctx, int r, int c, double dwell_t, double dt)
------------------------------------------------------------------------
调用时机：
  在 DWELL 阶段，每个 tick 都调用一次。

参数：
  dwell_t : 当前格子已经悬停了多久（秒）
  dt      : 本次 tick 的周期（秒）

适合做什么：
  - 持续累计数字候选
  - 持续读取视觉结果
  - 做 yaw 动态修正
  - 判断是否触发某种规则
  - 做 dwell 期间的控制逻辑

示例：
  void on_cell_dwell(Context& ctx, int r, int c, double dwell_t, double dt) override {
    if (dwell_t >= 1.0) {
      update_number_candidate(ctx, r, c);
    }
    if (dwell_t >= 0.1) {
      consume_vision_rule(ctx, r, c);
    }
  }


5) on_cell_leave(Context& ctx, int r, int c)
--------------------------------------------
调用时机：
  当前格子的 dwell 时间达到 dwell_s，准备离开该格子时调用一次。

适合做什么：
  - 最终确认识别结果
  - 将候选结果落库
  - 关闭视觉
  - 判断是否设置提前结束
  - 更新任务状态

示例：
  void on_cell_leave(Context& ctx, int r, int c) override {
    finalize_number_for_cell_once(ctx, r, c);
    ctx.vision_enable = false;
  }


6) on_task_finish(Context& ctx)
-------------------------------
调用时机：
  整个网格遍历结束时调用一次。
  包括：
  - 正常遍历完所有格子
  - should_finish_early() 返回 true

适合做什么：
  - 统一收尾
  - 关闭视觉
  - 打印总结日志
  - 整理最终状态


7) should_finish_early(Context& ctx)
------------------------------------
调用时机：
  每个 tick 开始阶段都会检查一次。

作用：
  是否需要提前结束整个 GridGotoDwellTask。

返回值：
  true  -> 立即结束，tick 返回 SUCCESS
  false -> 继续遍历

适合做什么：
  - 红色圆触发提前返航
  - 某个目标已经找到
  - 某个全局任务条件已经满足
  - 需要中断当前网格遍历

示例：
  bool should_finish_early(Context& ctx) override {
    return early_finish_;
  }


【七、最常见的“新任务接入方法”】
----------------------------------------------------------------------

情况 A：每个格子都要识别数字
--------------------------------
用法：
  on_cell_enter()  -> 清空候选
  on_cell_dwell()  -> 持续累计候选
  on_cell_leave()  -> 最终落库

情况 B：每个格子都要识别颜色/图形
--------------------------------
用法：
  on_cell_dwell() 中持续读视觉
  on_cell_leave() 中根据结果更新任务状态

情况 C：识别到某种目标后提前结束
--------------------------------
用法：
  on_cell_dwell() 或 on_cell_leave() 设置 early_finish_ = true
  should_finish_early() 返回 early_finish_

情况 D：识别到特殊格子后触发局部扫描
-----------------------------------
用法：
  on_cell_leave() 中设置一个“待扫描标志”
  然后由更高层任务或子扫描器接管
注意：
  不建议把复杂的扫描子状态机直接塞回 GridGotoDwellTask


【八、子类编写建议】
----------------------------------------------------------------------
建议只把“通用骨架”放在 GridGotoDwellTask 中。
子类只放“比赛规则”。

推荐分层：
  GridGotoDwellTask     -> 通用遍历骨架
  ContestGridTask       -> 本题规则
  QRGridTask            -> 下一题二维码规则
  ThermalGridTask       -> 下一题测温规则

这样换任务时，只需要换子类，不需要改底层遍历骨架。


【九、一个最小继承示例】
----------------------------------------------------------------------
class ContestGridTask : public GridGotoDwellTask {
public:
  using GridGotoDwellTask::GridGotoDwellTask;

protected:
  void on_task_enter(Context& ctx) override {
    ctx.vision_enable = false;
  }

  void on_cell_enter(Context& ctx, int r, int c) override {
    ctx.vision_enable = true;
  }

  void on_cell_dwell(Context& ctx, int r, int c, double dwell_t, double dt) override {
    // 在这里做识别逻辑
  }

  void on_cell_leave(Context& ctx, int r, int c) override {
    ctx.vision_enable = false;
  }

  bool should_finish_early(Context& ctx) override {
    return false;
  }
};


【十、一句话总结】
----------------------------------------------------------------------
GridGotoDwellTask = “走格子 + 到点稳定 + 悬停”的通用骨架
子类 = “这个格子里到底做什么”的具体业务逻辑

写新任务时：
不要改骨架，
优先通过重写 on_cell_enter / on_cell_dwell / on_cell_leave /
should_finish_early 这些接口来插入你的新规则。
======================================================================
*/
GridGotoDwellTask::GridGotoDwellTask(rclcpp::Logger lg,
                                     Grid& grid,
                                     double wp_xy_tol,
                                     double wp_z_tol,
                                     double v_xy_tol,
                                     double v_z_tol,
                                     int stable_required,
                                     double dwell_s)
: lg_(lg),
  grid_(grid),
  wp_xy_tol_(wp_xy_tol),
  wp_z_tol_(wp_z_tol),
  v_xy_tol_(v_xy_tol),
  v_z_tol_(v_z_tol),
  stable_required_(stable_required),
  dwell_s_(dwell_s)
{}

std::string GridGotoDwellTask::name() const
{
  return "GRID_GOTO_DWELL";
}

void GridGotoDwellTask::ensure_grid_ready(Context& ctx)
{
  if (grid_.waypoints.empty()) {
    grid_.build(ctx);
    RCLCPP_INFO(lg_, "Grid built: %zu waypoints", grid_.waypoints.size());
  }
}

void GridGotoDwellTask::onEnter(Context& ctx)
{
  stable_counter_ = 0;
  in_dwell_ = false;
  dwell_t_ = 0.0;

  ensure_grid_ready(ctx);
  grid_.wp_idx = 0;

  on_task_enter(ctx);
}

void GridGotoDwellTask::set_hover_sp(Context& ctx, float x, float y)
{
  ctx.use_vel_ctrl = false;
  ctx.sp_x = x;
  ctx.sp_y = y;
  ctx.sp_z = ctx.takeoff_z;
  ctx.sp_yaw = ctx.home_yaw;
}

bool GridGotoDwellTask::reached_and_stable(Context& ctx, float tx, float ty)
{
  const bool xy_ok =
    (Grid::dist2d(ctx.cx(), ctx.cy(), tx, ty) <= static_cast<float>(wp_xy_tol_));

  const bool z_ok =
    Grid::near(ctx.cz(), ctx.takeoff_z, static_cast<float>(wp_z_tol_));

  const float vxy = std::sqrt(ctx.local_pos.vx * ctx.local_pos.vx +
                              ctx.local_pos.vy * ctx.local_pos.vy);
  const float vz  = std::fabs(ctx.local_pos.vz);

  const bool v_ok =
    (vxy < static_cast<float>(v_xy_tol_)) &&
    (vz  < static_cast<float>(v_z_tol_));

  if (xy_ok && z_ok && v_ok) {
    stable_counter_++;
  } else {
    stable_counter_ = 0;
  }

  return stable_counter_ >= stable_required_;
}

ITask::Status GridGotoDwellTask::tick(Context& ctx, double dt)
{
  if (grid_.waypoints.empty()) {
    on_task_finish(ctx);
    return Status::SUCCESS;
  }

  grid_.update_current_cell(ctx);

  if (should_finish_early(ctx)) {
    RCLCPP_INFO(lg_, "Early finish requested");
    on_task_finish(ctx);
    return Status::SUCCESS;
  }

  if (grid_.wp_idx >= grid_.waypoints.size()) {
    RCLCPP_INFO(lg_, "Grid traversal done");
    on_task_finish(ctx);
    return Status::SUCCESS;
  }

  const auto& wp = grid_.waypoints[grid_.wp_idx];
  const float tx = wp.first;
  const float ty = wp.second;

  on_before_goto(ctx, ctx.current_r, ctx.current_c, tx, ty);
  set_hover_sp(ctx, tx, ty);

  // ===== DWELL 阶段 =====
  if (in_dwell_) {
    dwell_t_ += dt;

    on_cell_dwell(ctx, ctx.current_r, ctx.current_c, dwell_t_, dt);

    if (dwell_t_ >= dwell_s_) {
      on_cell_leave(ctx, ctx.current_r, ctx.current_c);

      in_dwell_ = false;
      dwell_t_ = 0.0;
      stable_counter_ = 0;
      grid_.wp_idx++;
    }

    return Status::RUNNING;
  }

  // ===== GOTO 阶段 =====
  if (reached_and_stable(ctx, tx, ty)) {
    stable_counter_ = 0;
    in_dwell_ = true;
    dwell_t_ = 0.0;

    grid_.update_current_cell(ctx);
    on_cell_enter(ctx, ctx.current_r, ctx.current_c);

    RCLCPP_INFO(lg_, "Arrived cell(%d,%d) -> DWELL", ctx.current_r, ctx.current_c);
  }

  return Status::RUNNING;
}