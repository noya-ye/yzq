#pragma once
#include <string>

struct Context;

class ITask {
public:
  enum class Status {
    RUNNING,
    SUCCESS,
    FAILURE
  };

  virtual ~ITask() = default;

  virtual std::string name() const = 0;

  // 第一次进入任务
  virtual void onEnter(Context& ctx) {}

  // 周期执行任务
  virtual Status tick(Context& ctx, double dt_s) = 0;

  // 正常完成或者失败退出
  virtual void onExit(Context& ctx) {}

  // 是否允许被暂停
  virtual bool canPause(const Context& ctx) const {
    (void)ctx;
    return true;
  }

  // 被中断时调用：保存状态，不清空任务进度
  virtual void onPause(Context& ctx) {
    (void)ctx;
  }

  // 从中断状态恢复时调用：继续之前的状态
  virtual void onResume(Context& ctx) {
    (void)ctx;
  }

  // 被强制取消时调用：清理状态，不会再回到这个任务
  virtual void onCancel(Context& ctx) {
    (void)ctx;
  }
};