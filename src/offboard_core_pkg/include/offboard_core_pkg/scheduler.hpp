#pragma once
#include <memory>
#include <vector>
#include <string>
#include "offboard_core_pkg/itask.hpp"

struct Context;

class Scheduler {
public:
  void add(std::unique_ptr<ITask> t) { tasks_.emplace_back(std::move(t)); }

  void reset() {
    idx_ = 0;
    entered_ = false;
    done_ = false;
  }

  bool done() const { return done_; }

  std::string current_name() const {
    if (done_ || idx_ >= tasks_.size()) return "DONE";
    return tasks_[idx_]->name();
  }

  int current_index() const {
    if (done_) {
      return static_cast<int>(tasks_.size());
    }
    return static_cast<int>(idx_);
  }

  int total_count() const {
    return static_cast<int>(tasks_.size());
  }

  void tick(Context& ctx, double dt_s) {
    if (done_) return;
    if (idx_ >= tasks_.size()) {
      done_ = true;
      return;
    }

    ITask* cur = tasks_[idx_].get();
    if (!entered_) {
      cur->onEnter(ctx);
      entered_ = true;
    }

    auto st = cur->tick(ctx, dt_s);
    if (st == ITask::Status::RUNNING) return;

    cur->onExit(ctx);
    entered_ = false;

    if (st == ITask::Status::FAILURE) {
      done_ = true;
      return;
    }

    idx_++;
    if (idx_ >= tasks_.size()) done_ = true;
  }

private:
  std::vector<std::unique_ptr<ITask>> tasks_;
  size_t idx_{0};
  bool entered_{false};
  bool done_{false};
};