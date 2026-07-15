#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/planners/ego_vel_planner.hpp"

namespace offboard_core_pkg
{

/*
 * 向EGO发布目标点，并跟踪EGO生成的连续轨迹。
 *
 * 控制链：
 *
 *   EgoGotoTask
 *        |
 *        | 发布 geometry_msgs/PoseStamped 目标
 *        v
 *   EGO 2D Planner
 *        |
 *        | 发布 PositionCommand
 *        v
 *   EgoVelPlanner
 *        |
 *        | 写入：
 *        |   position
 *        |   velocity feedforward
 *        |   acceleration feedforward
 *        v
 *   PX4 Position Controller
 */
class EgoGotoTask : public ITask
{
public:
  struct Config
  {
    std::string task_name{"EGO_GOTO"};
    std::string goal_name{"GOAL"};
    std::string goal_frame{"camera_init"};

    /*
     * 目标相对起飞点的PX4 local XY。
     *
     * height_m表示相对地面的飞行高度，
     * 不是PX4 NED绝对z坐标。
     */
    double x_rel{0.0};
    double y_rel{0.0};
    double height_m{1.5};

    // PX4 local yaw
    double yaw_local{0.0};

    // 到达判断
    double arrive_xy_m{0.15};
    double arrive_z_m{0.15};

    // 稳定判断
    double stable_vxy_mps{0.15};
    double stable_vz_mps{0.12};
    double stable_required_s{0.40};

    /*
     * 周期性重新发布目标，避免规划器重启或漏收目标。
     *
     * <= 0时可以由cpp实现为不周期重发。
     */
    double goal_republish_s{1.0};

    /*
     * 发布新目标后，不接受早于该时刻的旧PositionCommand。
     */
    double cmd_guard_s{0.30};

    // EGO轨迹转换参数
    EgoVelPlanner::Config planner;
  };

  EgoGotoTask(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<
      geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
    const Config& cfg);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(
    Context& ctx,
    double dt_s) override;

  void onExit(Context& ctx) override;

private:
  /*
   * 初始化本次EGO目标：
   *
   * 1. 计算PX4 local目标
   * 2. 转换为EGO世界系目标
   * 3. 发布目标
   * 4. 设置PositionCommand时间保护
   */
  bool startGoal(Context& ctx);

  // 发布EGO目标点
  void publishGoal();

  /*
   * 普通位置悬停。
   *
   * 该函数的cpp实现中应同时：
   *
   *   ctx.use_vel_ctrl = false;
   *   ctx.use_trajectory_ff = false;
   *
   * 并清理速度、加速度前馈。
   */
  void holdCurrent(Context& ctx);

  /*
   * 将PX4 local XY位移反向映射到EGO XY。
   *
   * 它应当与EgoVelPlanner::mapXY()保持互逆。
   */
  void inverseMapXY(
    double local_x,
    double local_y,
    double& ego_x,
    double& ego_y) const;

  static uint64_t nowUs();

  // ============================================================
  // 控制模式辅助函数
  // ============================================================

  /*
   * 开启EGO轨迹跟踪模式。
   *
   * 主控制器始终是PX4位置控制器。
   * velocity和acceleration只作为前馈。
   */
  static void enableTrajectoryControl(
    Context& ctx,
    const EgoVelPlanner::Config& planner_cfg)
  {
    ctx.use_vel_ctrl = false;

    ctx.use_trajectory_ff =
      planner_cfg.use_velocity_ff ||
      planner_cfg.use_acceleration_ff;
  }

  /*
   * 关闭EGO轨迹前馈，恢复普通PX4位置控制。
   *
   * 清零Context缓存，避免后续任务误用旧前馈。
   */
  static void disableTrajectoryControl(Context& ctx)
  {
    ctx.use_vel_ctrl = false;
    ctx.use_trajectory_ff = false;

    ctx.sp_vx = 0.0f;
    ctx.sp_vy = 0.0f;
    ctx.sp_vz = 0.0f;

    ctx.sp_ax = 0.0f;
    ctx.sp_ay = 0.0f;
    ctx.sp_az = 0.0f;
  }

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

  Config cfg_;
  EgoVelPlanner planner_;

  bool started_{false};
  bool finished_{false};

  double stable_elapsed_s_{0.0};
  double republish_elapsed_s_{0.0};

  // 只接受该时间之后的PositionCommand
  uint64_t accept_cmd_after_us_{0};

  // PX4 local目标
  double target_local_x_{0.0};
  double target_local_y_{0.0};
  double target_local_z_{0.0};

  // EGO世界系目标
  double target_ego_x_{0.0};
  double target_ego_y_{0.0};
  double target_ego_z_{0.0};
};

}  // namespace offboard_core_pkg