#pragma once
#include <string>

struct Context;

class ITask {
public:
  enum class Status { RUNNING, SUCCESS, FAILURE };
  virtual ~ITask() = default;

  virtual std::string name() const = 0;
  virtual void onEnter(Context& ctx) {}
  virtual Status tick(Context& ctx, double dt_s) = 0;
  virtual void onExit(Context& ctx) {}
};
