#pragma once

#include <memory>
#include <vector>
#include <string>
#include <limits>

#include "offboard_core_pkg/itask.hpp"

struct Context;

class Scheduler {
public:
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