// void on_timer()
// {
//   const auto t = now();
//   const double dt = (t - last_time_).seconds();
//   last_time_ = t;

//   // ============================================================
//   // 1. 正常发布 OffboardControlMode
//   // ============================================================
//   if (!sched_.done() && !ctx_.handover_to_px4_land) {
//     px4_->publish_offboard_control_mode(ctx_);
//   }

//   // ============================================================
//   // 2. 中断逻辑
//   // 注意：
//   // interrupt：临时插队，任务完成后回到原任务
//   // jumpTo：强制跳转，不再回到原任务
//   // ============================================================

//   // 示例 1：发现障碍物，临时进入避障任务
//   // 避障完成后会回到 SnakeGridTask 继续蛇形遍历
//   //
//   // if (ctx_.obstacle_too_close) {
//   //   sched_.interrupt("AVOID", ctx_);
//   // }

//   // 示例 2：发现目标，临时进入目标跟踪任务
//   //
//   // if (ctx_.vision_detected) {
//   //   sched_.interrupt("TRACK_TARGET", ctx_);
//   // }

//   // 示例 3：低电量 / 超时，强制返航
//   // 不再回到蛇形任务
//   //
//   // if (ctx_.low_battery || ctx_.mission_timeout) {
//   //   sched_.jumpTo("RETURN_HOME", ctx_);
//   // }

//   // 示例 4：严重异常，直接交给 PX4 降落
//   //
//   // if (ctx_.critical_error) {
//   //   sched_.jumpTo("EMERGENCY_LAND", ctx_);
//   // }

//   // ============================================================
//   // 3. 执行当前任务
//   // ============================================================
//   sched_.tick(ctx_, dt);

//   // ============================================================
//   // 4. 发布 setpoint
//   // ============================================================
//   if (!sched_.done()) {
//     px4_->publish_setpoint_from_ctx(ctx_);
//   }
// }

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <limits>

#include "offboard_core_pkg/itask.hpp"

struct Context;

class Scheduler {
public:
void clear()
{
  tasks_.clear();
  paused_stack_.clear();
  idx_ = 0;
  entered_ = false;
  done_ = true;
}
  // ============================================================
  // 兼容旧版本接口：当前任务序号，0-based
  // 旧代码里一般会用 current_index() + 1 显示
  // ============================================================
  size_t current_index() const
  {
    const size_t total = total_count();

    if (total == 0) {
      return 0;
    }

    if (done_) {
      return total - 1;
    }

    // 如果当前任务是主线任务，返回它在主线任务中的序号
    size_t seq_idx = 0;

    for (size_t i = 0; i < tasks_.size(); ++i) {
      if (!tasks_[i].in_sequence) {
        continue;
      }

      if (i == idx_) {
        return seq_idx;
      }

      ++seq_idx;
    }

    // 如果当前任务是 addAux 添加的中断任务，
    // 优先显示被中断的主线任务序号
    if (!paused_stack_.empty()) {
      const size_t paused_idx = paused_stack_.back().idx;

      seq_idx = 0;
      for (size_t i = 0; i < tasks_.size(); ++i) {
        if (!tasks_[i].in_sequence) {
          continue;
        }

        if (i == paused_idx) {
          return seq_idx;
        }

        ++seq_idx;
      }
    }

    // 兜底：防止 current_wp 显示成 total + 1
    return total - 1;
  }

  // ============================================================
  // 兼容旧版本接口：主线任务总数
  // 注意：addAux 添加的中断任务不计入 total_count
  // ============================================================
  size_t total_count() const
  {
    size_t count = 0;

    for (const auto& slot : tasks_) {
      if (slot.in_sequence) {
        ++count;
      }
    }

    return count;
  }
  enum class InterruptMode {
    RESUME_CURRENT,   // 插队任务完成后，回到被中断任务
    DROP_CURRENT      // 丢弃当前任务，不再回来
  };

  void add(std::unique_ptr<ITask> t) {
    tasks_.push_back(TaskSlot{std::move(t), true});
  }

  // 辅助任务：不会在线性流程里自动执行，只能 interrupt / jumpTo 调用
  void addAux(std::unique_ptr<ITask> t) {
    tasks_.push_back(TaskSlot{std::move(t), false});
  }

  void reset() {
    idx_ = firstSequenceIndex();
    entered_ = false;
    done_ = tasks_.empty() || idx_ == npos();
    paused_stack_.clear();
  }

  bool done() const {
    return done_;
  }

  std::string current_name() const {
    if (done_ || idx_ >= tasks_.size()) {
      return "DONE";
    }
    return tasks_[idx_].task->name();
  }

  bool hasPausedTask() const {
    return !paused_stack_.empty();
  }

  // ============================================================
  // 核心：中断当前任务，切到另一个任务
  // 插队任务结束后，会自动回到原任务
  // ============================================================
  bool interrupt(
    const std::string& task_name,
    Context& ctx,
    InterruptMode mode = InterruptMode::RESUME_CURRENT)
  {
    const size_t target = findTask(task_name);
    if (target == npos()) {
      return false;
    }

    if (!done_ && idx_ < tasks_.size()) {
      ITask* cur = tasks_[idx_].task.get();

      // 如果已经是当前任务，不需要重复中断
      if (cur->name() == task_name) {
        return true;
      }

      if (entered_) {
        if (!cur->canPause(ctx)) {
          return false;
        }

        if (mode == InterruptMode::RESUME_CURRENT) {
          cur->onPause(ctx);
          paused_stack_.push_back(Frame{idx_, true});
        } else {
          cur->onCancel(ctx);
        }
      } else {
        if (mode == InterruptMode::RESUME_CURRENT) {
          paused_stack_.push_back(Frame{idx_, false});
        }
      }
    }

    idx_ = target;
    entered_ = false;
    done_ = false;
    return true;
  }

  // ============================================================
  // 强制跳转：取消当前任务和所有暂停任务，不再恢复
  // 适合：低电量返航、紧急降落、任务彻底切换
  // ============================================================
  bool jumpTo(const std::string& task_name, Context& ctx) {
    const size_t target = findTask(task_name);
    if (target == npos()) {
      return false;
    }

    cancelCurrentAndPaused(ctx);

    idx_ = target;
    entered_ = false;
    done_ = false;
    return true;
  }

  void tick(Context& ctx, double dt_s) {
    if (done_) {
      return;
    }

    if (idx_ >= tasks_.size()) {
      done_ = true;
      return;
    }

    ITask* cur = tasks_[idx_].task.get();

    if (!entered_) {
      cur->onEnter(ctx);
      entered_ = true;
    }

    const auto st = cur->tick(ctx, dt_s);

    if (st == ITask::Status::RUNNING) {
      return;
    }

    cur->onExit(ctx);
    entered_ = false;

    if (st == ITask::Status::FAILURE) {
      done_ = true;
      return;
    }

    // 如果有被中断的任务，优先恢复它
    if (!paused_stack_.empty()) {
      const Frame frame = paused_stack_.back();
      paused_stack_.pop_back();

      idx_ = frame.idx;

      if (idx_ >= tasks_.size()) {
        done_ = true;
        return;
      }

      if (frame.was_entered) {
        tasks_[idx_].task->onResume(ctx);
        entered_ = true;
      } else {
        entered_ = false;
      }

      done_ = false;
      return;
    }

    // 否则继续线性流程
    advanceToNextSequence();
  }

private:
  struct TaskSlot {
    std::unique_ptr<ITask> task;
    bool in_sequence{true};
  };

  struct Frame {
    size_t idx{0};
    bool was_entered{false};
  };

  static constexpr size_t npos() {
    return std::numeric_limits<size_t>::max();
  }

  size_t findTask(const std::string& name) const {
    for (size_t i = 0; i < tasks_.size(); ++i) {
      if (tasks_[i].task && tasks_[i].task->name() == name) {
        return i;
      }
    }
    return npos();
  }

  size_t firstSequenceIndex() const {
    for (size_t i = 0; i < tasks_.size(); ++i) {
      if (tasks_[i].in_sequence) {
        return i;
      }
    }
    return npos();
  }

  void advanceToNextSequence() {
    size_t next = idx_ + 1;

    while (next < tasks_.size() && !tasks_[next].in_sequence) {
      ++next;
    }

    if (next >= tasks_.size()) {
      done_ = true;
      return;
    }

    idx_ = next;
    entered_ = false;
    done_ = false;
  }

  void cancelCurrentAndPaused(Context& ctx) {
    if (!done_ && idx_ < tasks_.size() && entered_) {
      tasks_[idx_].task->onCancel(ctx);
    }

    while (!paused_stack_.empty()) {
      const Frame frame = paused_stack_.back();
      paused_stack_.pop_back();

      if (frame.idx < tasks_.size() && frame.was_entered) {
        tasks_[frame.idx].task->onCancel(ctx);
      }
    }

    entered_ = false;
  }

private:
  std::vector<TaskSlot> tasks_;
  std::vector<Frame> paused_stack_;

  size_t idx_{0};
  bool entered_{false};
  bool done_{false};
};



// 使用方式

// 你的线性主流程可以这样写：

// void build_scheduler() {
//   sched_.reset();

//   sched_.add(std::make_unique<WaitHomeTask>(get_logger()));
//   sched_.add(std::make_unique<PresetpointTask>(get_logger()));
//   sched_.add(std::make_unique<SetOffboardTask>(get_logger(), *px4_));
//   sched_.add(std::make_unique<ArmTask>(get_logger(), *px4_));
//   sched_.add(std::make_unique<TakeoffTask>(
//     get_logger(), *px4_, arrival_error_max_));

//   sched_.add(std::make_unique<MissionTask>(get_logger(), *px4_));

//   sched_.add(std::make_unique<HomeStabilizeTask>(
//     get_logger(), home_stabilize_s_));

//   sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(
//     get_logger(), *px4_));

//   // 辅助任务，不会自动执行，只能被 interrupt / jumpTo 调用
//   sched_.addAux(std::make_unique<AvoidTask>(get_logger()));
//   sched_.addAux(std::make_unique<ReturnHomeTask>(get_logger()));
//   sched_.addAux(std::make_unique<EmergencyLandTask>(get_logger(), *px4_));
// }