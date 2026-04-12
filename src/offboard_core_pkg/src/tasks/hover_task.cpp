#include "offboard_core_pkg/tasks/hover_task.hpp"

HoverTask::HoverTask(rclcpp::Logger lg, double hover_time_s)
: lg_(lg), hover_time_s_(hover_time_s)
{}

std::string HoverTask::name() const
{
    return "HOVER";
}

void HoverTask::onEnter(Context& ctx)
{
    t_ = 0.0;

    RCLCPP_INFO(lg_, "HOVER enter: %.2f seconds", hover_time_s_);
}

ITask::Status HoverTask::tick(Context& ctx, double dt)
{
    ctx.use_vel_ctrl = false;

    // 保持当前位置悬停
    ctx.sp_x = ctx.cx();
    ctx.sp_y = ctx.cy();
    ctx.sp_z = ctx.cz();
    ctx.sp_yaw = ctx.home_yaw;

    t_ += dt;

    if (t_ >= hover_time_s_) {
        RCLCPP_INFO(lg_, "HOVER done");
        return Status::SUCCESS;
    }

    return Status::RUNNING;
}