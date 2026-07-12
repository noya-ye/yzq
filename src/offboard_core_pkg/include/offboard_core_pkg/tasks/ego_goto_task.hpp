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

class EgoGotoTask : public ITask
{
public:
  struct Config
  {
    std::string task_name{"EGO_GOTO"};
    std::string goal_name{"GOAL"};
    std::string goal_frame{"camera_init"};

    // 目标相对起飞点的PX4 local XY，以及相对地面的目标高度。
    double x_rel{0.0};
    double y_rel{0.0};
    double height_m{1.5};
    double yaw_local{0.0};

    double arrive_xy_m{0.15};
    double arrive_z_m{0.15};
    double stable_vxy_mps{0.15};
    double stable_vz_mps{0.12};
    double stable_required_s{0.40};

    double goal_republish_s{1.0};
    double cmd_guard_s{0.30};

    EgoVelPlanner::Config planner;
  };

  EgoGotoTask(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub,
    const Config& cfg);

  std::string name() const override;
  void onEnter(Context& ctx) override;
  Status tick(Context& ctx, double dt_s) override;
  void onExit(Context& ctx) override;

private:
  bool startGoal(Context& ctx);
  void publishGoal();
  void holdCurrent(Context& ctx);
  void inverseMapXY(double local_x, double local_y, double& ego_x, double& ego_y) const;
  static uint64_t nowUs();

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

  Config cfg_;
  EgoVelPlanner planner_;

  bool started_{false};
  bool finished_{false};
  double stable_elapsed_s_{0.0};
  double republish_elapsed_s_{0.0};
  uint64_t accept_cmd_after_us_{0};

  double target_local_x_{0.0};
  double target_local_y_{0.0};
  double target_local_z_{0.0};
  double target_ego_x_{0.0};
  double target_ego_y_{0.0};
  double target_ego_z_{0.0};
};

}  // namespace offboard_core_pkg