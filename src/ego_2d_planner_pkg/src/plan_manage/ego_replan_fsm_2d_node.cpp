#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ego_2d_planner_pkg/msg/bspline2_d.hpp"

#include "ego_2d_planner_pkg/common/types.hpp"
#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"
#include "ego_2d_planner_pkg/plan_manage/planner_manager_2d.hpp"
#include "ego_2d_planner_pkg/bspline_opt/uniform_bspline_2d.hpp"

using ego_2d_planner_pkg::Vec2;
using ego_2d_planner_pkg::PlannerParams2D;
using ego_2d_planner_pkg::GridMap2D;
using ego_2d_planner_pkg::PlannerManager2D;
using ego_2d_planner_pkg::UniformBspline2D;
using ego_2d_planner_pkg::msg::Bspline2D;

class EgoReplanFSM2DNode : public rclcpp::Node
{
public:
  enum FSMExecState
  {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP
  };

  EgoReplanFSM2DNode() : Node("ego_replan_fsm_2d_node")
  {
    loadParams();

    map_.configure(p_);
    manager_.setParams(p_);

    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>("/ego_2d_planner/raw_path", 10);
    smooth_path_pub_ = create_publisher<nav_msgs::msg::Path>("/ego_2d_planner/smooth_path", 10);
    selected_path_pub_ = create_publisher<nav_msgs::msg::Path>("/ego_2d_planner/selected_path", 10);
    bspline_pub_ = create_publisher<Bspline2D>("/ego_2d_planner/bspline_2d", 10);
    local_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/simple_2d_planner/local_goal", 10);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/ego_2d_planner/occupancy_grid", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ego_2d_planner/local_goal_marker", 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      p_.cloud_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&EgoReplanFSM2DNode::cloudCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      p_.odom_topic,
      20,
      std::bind(&EgoReplanFSM2DNode::odomCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      p_.goal_topic,
      10,
      std::bind(&EgoReplanFSM2DNode::goalCallback, this, std::placeholders::_1));

    const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, p_.planning_rate));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&EgoReplanFSM2DNode::execFSMCallback, this));

    last_replan_time_ = now();
    emergency_start_time_ = now();

    RCLCPP_WARN(get_logger(), "ego_replan_fsm_2d_node started, EGO-style FSM enabled");
    RCLCPP_WARN(get_logger(), "Input : %s + %s + %s",
                p_.cloud_topic.c_str(), p_.odom_topic.c_str(), p_.goal_topic.c_str());
    RCLCPP_WARN(get_logger(), "Output: /ego_2d_planner/bspline_2d + /simple_2d_planner/local_goal");
  }

private:
  void loadParams()
  {
    p_.frame_id = declare_parameter<std::string>("frame_id", p_.frame_id);
    p_.cloud_topic = declare_parameter<std::string>("cloud_topic", p_.cloud_topic);
    p_.odom_topic = declare_parameter<std::string>("odom_topic", p_.odom_topic);
    p_.goal_topic = declare_parameter<std::string>("goal_topic", p_.goal_topic);

    p_.resolution = declare_parameter<double>("grid_map/resolution", p_.resolution);
    p_.map_size_x = declare_parameter<double>("grid_map/map_size_x", p_.map_size_x);
    p_.map_size_y = declare_parameter<double>("grid_map/map_size_y", p_.map_size_y);
    p_.cloud_min_z = declare_parameter<double>("grid_map/cloud_min_z", p_.cloud_min_z);
    p_.cloud_max_z = declare_parameter<double>("grid_map/cloud_max_z", p_.cloud_max_z);
    p_.inflate_radius = declare_parameter<double>("grid_map/inflate_radius", p_.inflate_radius);
    p_.occupied_threshold = declare_parameter<int>("grid_map/occupied_threshold", p_.occupied_threshold);
    p_.persistent_cache_enable = declare_parameter<bool>("grid_map/persistent_cache_enable", p_.persistent_cache_enable);
    p_.persistent_confirm_s = declare_parameter<double>("grid_map/persistent_confirm_s", p_.persistent_confirm_s);
    p_.persistent_miss_tolerance_s = declare_parameter<double>("grid_map/persistent_miss_tolerance_s", p_.persistent_miss_tolerance_s);
    p_.persistent_min_observations = declare_parameter<int>("grid_map/persistent_min_observations", p_.persistent_min_observations);

    p_.planning_rate = declare_parameter<double>("fsm/planning_rate", p_.planning_rate);
    p_.thresh_replan_time = declare_parameter<double>("fsm/thresh_replan_time", p_.thresh_replan_time);
    p_.thresh_replan_dist = declare_parameter<double>("fsm/thresh_replan_dist", p_.thresh_replan_dist);
    p_.thresh_no_replan_dist = declare_parameter<double>("fsm/thresh_no_replan_dist", p_.thresh_no_replan_dist);
    p_.target_reached_tol = declare_parameter<double>("fsm/target_reached_tol", p_.target_reached_tol);
    p_.emergency_time = declare_parameter<double>("fsm/emergency_time", p_.emergency_time);
    p_.max_fsm_plan_failures = declare_parameter<int>("fsm/max_fsm_plan_failures", p_.max_fsm_plan_failures);

    p_.astar_max_iter = declare_parameter<int>("search/astar_max_iter", p_.astar_max_iter);

    p_.control_points_distance = declare_parameter<double>("manager/control_points_distance", p_.control_points_distance);
    p_.bspline_sample_step = declare_parameter<double>("manager/bspline_sample_step", p_.bspline_sample_step);
    p_.collision_check_step = declare_parameter<double>("manager/collision_check_step", p_.collision_check_step);
    p_.fallback_to_raw_path = declare_parameter<bool>("manager/fallback_to_raw_path", p_.fallback_to_raw_path);
    p_.hover_if_plan_failed = declare_parameter<bool>("manager/hover_if_plan_failed", p_.hover_if_plan_failed);
    p_.lookahead_dist = declare_parameter<double>("manager/lookahead_dist", p_.lookahead_dist);
    p_.fixed_z = declare_parameter<double>("manager/fixed_z", p_.fixed_z);

    p_.optimizer_enable = declare_parameter<bool>("optimization/enable", p_.optimizer_enable);
    p_.optimizer_max_iter = declare_parameter<int>("optimization/max_iter", p_.optimizer_max_iter);
    p_.optimizer_step_size = declare_parameter<double>("optimization/step_size", p_.optimizer_step_size);
    p_.optimizer_max_update = declare_parameter<double>("optimization/max_update", p_.optimizer_max_update);
    p_.dist0 = declare_parameter<double>("optimization/dist0", p_.dist0);
    p_.max_vel = declare_parameter<double>("optimization/max_vel", p_.max_vel);
    p_.max_acc = declare_parameter<double>("optimization/max_acc", p_.max_acc);
    p_.knot_span = declare_parameter<double>("optimization/knot_span", p_.knot_span);
    p_.lambda_smooth = declare_parameter<double>("optimization/lambda_smooth", p_.lambda_smooth);
    p_.lambda_collision = declare_parameter<double>("optimization/lambda_collision", p_.lambda_collision);
    p_.lambda_feasibility = declare_parameter<double>("optimization/lambda_feasibility", p_.lambda_feasibility);
    p_.lambda_fitness = declare_parameter<double>("optimization/lambda_fitness", p_.lambda_fitness);
    p_.max_rebound_attempts = declare_parameter<int>("optimization/max_rebound_attempts", p_.max_rebound_attempts);
    p_.retry_collision_scale = declare_parameter<double>("optimization/retry_collision_scale", p_.retry_collision_scale);
    p_.retry_smooth_scale = declare_parameter<double>("optimization/retry_smooth_scale", p_.retry_smooth_scale);
    p_.retry_dist0_scale = declare_parameter<double>("optimization/retry_dist0_scale", p_.retry_dist0_scale);
  }

  std::string stateName(FSMExecState s) const
  {
    switch (s) {
      case INIT: return "INIT";
      case WAIT_TARGET: return "WAIT_TARGET";
      case GEN_NEW_TRAJ: return "GEN_NEW_TRAJ";
      case REPLAN_TRAJ: return "REPLAN_TRAJ";
      case EXEC_TRAJ: return "EXEC_TRAJ";
      case EMERGENCY_STOP: return "EMERGENCY_STOP";
      default: return "UNKNOWN";
    }
  }

  void changeFSMExecState(FSMExecState new_state, const std::string& reason)
  {
    if (exec_state_ == new_state) {
      return;
    }

    RCLCPP_WARN(get_logger(), "[FSM] %s -> %s, reason=%s",
                stateName(exec_state_).c_str(), stateName(new_state).c_str(), reason.c_str());

    exec_state_ = new_state;
    if (new_state == EMERGENCY_STOP) {
      emergency_start_time_ = now();
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_ = Vec2{msg->pose.pose.position.x, msg->pose.pose.position.y};
    current_z_ = msg->pose.pose.position.z;
    have_odom_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const Vec2 new_goal{msg->pose.position.x, msg->pose.position.y};

    if (!have_goal_ || ego_2d_planner_pkg::dist(new_goal, goal_pos_) > 1e-3) {
      goal_pos_ = new_goal;
      have_goal_ = true;
      have_new_target_ = true;
      plan_fail_count_ = 0;

      RCLCPP_WARN(get_logger(), "[GOAL] new target x=%.2f y=%.2f", goal_pos_.x, goal_pos_.y);

      if (exec_state_ == WAIT_TARGET || exec_state_ == EXEC_TRAJ || exec_state_ == EMERGENCY_STOP) {
        changeFSMExecState(GEN_NEW_TRAJ, "TARGET");
      }
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!have_odom_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[MAP] waiting odom before building local grid");
      return;
    }

    map_.resetAround(current_pos_);
    map_.beginUpdate(now().seconds());

    std::size_t used = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const double x = static_cast<double>(*iter_x);
        const double y = static_cast<double>(*iter_y);
        const double z = static_cast<double>(*iter_z);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        if (z < p_.cloud_min_z || z > p_.cloud_max_z) continue;
        map_.setOccupiedWorld(Vec2{x, y});
        ++used;
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "[MAP] PointCloud2 iterator error: %s", e.what());
      return;
    }

    map_.finishUpdate();
    map_.inflateObstacles();
    map_.computeDistanceField();
    have_map_ = true;
    publishGrid();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[MAP] cloud used=%zu grid=%dx%d res=%.2f origin=(%.2f %.2f) cache=%zu/%zu",
      used, map_.width(), map_.height(), map_.resolution(), map_.origin_x(), map_.origin_y(),
      map_.persistentCellCount(), map_.persistentCandidateCount());
  }

  void execFSMCallback()
  {
    switch (exec_state_) {
      case INIT:
      {
        if (!have_odom_ || !have_map_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                               "[FSM][INIT] waiting odom=%d map=%d", have_odom_, have_map_);
          return;
        }
        changeFSMExecState(WAIT_TARGET, "INIT_OK");
        return;
      }

      case WAIT_TARGET:
      {
        if (!have_goal_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "[FSM][WAIT_TARGET] waiting goal");
          return;
        }
        changeFSMExecState(GEN_NEW_TRAJ, "HAVE_TARGET");
        return;
      }

      case GEN_NEW_TRAJ:
      {
        if (!readyToPlan()) return;
        const auto result = manager_.planGlobalTraj(map_, current_pos_, goal_pos_);
        handlePlanResult(result, true);
        return;
      }

      case REPLAN_TRAJ:
      {
        if (!readyToPlan()) return;
        const auto result = manager_.reboundReplan(map_, current_pos_, goal_pos_, false);
        handlePlanResult(result, false);
        return;
      }

      case EXEC_TRAJ:
      {
        execTrajCallback();
        return;
      }

      case EMERGENCY_STOP:
      {
        emergencyStopCallback();
        return;
      }
    }
  }

  bool readyToPlan() 
  {
    if (!have_odom_ || !have_map_ || !have_goal_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[PLAN] waiting odom=%d map=%d goal=%d", have_odom_, have_map_, have_goal_);
      return false;
    }
    return true;
  }

  void handlePlanResult(const PlannerManager2D::PlanResult& result, bool init)
  {
    publishPath(result.raw_path, raw_path_pub_);
    publishPath(result.smooth_path, smooth_path_pub_);

    if (result.success) {
      selected_path_ = result.selected_path;
      have_exec_path_ = !selected_path_.empty();
      have_new_target_ = false;
      plan_fail_count_ = 0;
      last_replan_time_ = now();

      publishPath(selected_path_, selected_path_pub_);
      publishBspline2D(result);

      RCLCPP_WARN(get_logger(),
                  "[PLAN_OK] %s raw=%zu smooth=%zu attempt=%d cost=%.2f->%.2f",
                  result.message.c_str(), result.raw_path.size(), result.smooth_path.size(),
                  result.rebound_attempt, result.init_cost, result.final_cost);

      changeFSMExecState(EXEC_TRAJ, init ? "GEN_NEW_SUCCESS" : "REPLAN_SUCCESS");
      return;
    }

    ++plan_fail_count_;
    RCLCPP_WARN(get_logger(),
                "[PLAN_FAIL] state=%s fail_count=%d/%d msg=%s raw_safe=%d smooth_safe=%d",
                init ? "GEN_NEW_TRAJ" : "REPLAN_TRAJ",
                plan_fail_count_, p_.max_fsm_plan_failures,
                result.message.c_str(), result.raw_path_safe, result.smooth_safe);

    // EGO-style: failed optimized trajectory is not accepted. Rebound/replan first;
    // only after repeated failures, enter emergency hover.
    if (plan_fail_count_ >= p_.max_fsm_plan_failures) {
      changeFSMExecState(EMERGENCY_STOP, "PLAN_FAIL_LIMIT");
    } else {
      changeFSMExecState(init ? GEN_NEW_TRAJ : REPLAN_TRAJ, "PLAN_RETRY");
    }
  }

  void execTrajCallback()
  {
    if (!readyToPlan()) return;

    if (have_new_target_) {
      changeFSMExecState(GEN_NEW_TRAJ, "NEW_TARGET_DURING_EXEC");
      return;
    }

    if (!have_exec_path_ || selected_path_.size() < 2) {
      changeFSMExecState(GEN_NEW_TRAJ, "EMPTY_EXEC_PATH");
      return;
    }

    if (ego_2d_planner_pkg::dist(current_pos_, goal_pos_) < p_.target_reached_tol) {
      publishLocalGoal(goal_pos_);
      publishMarker(goal_pos_);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[FSM][EXEC_TRAJ] target reached, hold target");
      return;
    }

    if (!map_.isPathSafe(selected_path_, p_.collision_check_step)) {
      RCLCPP_WARN(get_logger(), "[TRAJ_CHECK] current optimized path unsafe -> REPLAN_TRAJ");
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
      return;
    }

    const double time_since_replan = (now() - last_replan_time_).seconds();
    const double remain_dist = ego_2d_planner_pkg::dist(current_pos_, goal_pos_);
    if (time_since_replan > p_.thresh_replan_time && remain_dist > p_.thresh_no_replan_dist) {
      changeFSMExecState(REPLAN_TRAJ, "PERIODIC_REPLAN");
      return;
    }

    const Vec2 local_goal = UniformBspline2D::lookaheadPoint(selected_path_, current_pos_, p_.lookahead_dist);
    publishLocalGoal(local_goal);
    publishMarker(local_goal);
    publishPath(selected_path_, selected_path_pub_);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "[EXEC] local_goal=(%.2f %.2f) cur=(%.2f %.2f) remain=%.2f",
                         local_goal.x, local_goal.y, current_pos_.x, current_pos_.y, remain_dist);
  }

  void emergencyStopCallback()
  {
    // In this 2D planner, emergency stop means: publish current position as local goal.
    // No raw path or unsafe optimized trajectory is allowed to become a forward command.
    if (p_.hover_if_plan_failed && have_odom_) {
      publishLocalGoal(current_pos_);
      publishMarker(current_pos_);
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "[EMERGENCY_STOP] hover at current position; wait %.2fs then try replan",
                         p_.emergency_time);

    if (have_goal_ && have_map_ && have_odom_ &&
        (now() - emergency_start_time_).seconds() > p_.emergency_time) {
      plan_fail_count_ = 0;
      changeFSMExecState(GEN_NEW_TRAJ, "EMERGENCY_RETRY");
    }
  }


  void publishBspline2D(const PlannerManager2D::PlanResult& result)
  {
    if (!result.success || result.selected_ctrl_pts.size() < 4) {
      return;
    }

    Bspline2D msg;
    msg.header.stamp = now();
    msg.header.frame_id = p_.frame_id;
    msg.start_time = msg.header.stamp;
    msg.traj_id = ++bspline_traj_id_;
    msg.order = result.bspline_order;
    msg.knot_span = result.knot_span;
    msg.duration = result.bspline_duration;
    msg.fixed_z = p_.fixed_z;

    msg.pos_pts.reserve(result.selected_ctrl_pts.size());
    for (const auto& c : result.selected_ctrl_pts) {
      geometry_msgs::msg::Point pt;
      pt.x = c.x;
      pt.y = c.y;
      pt.z = p_.fixed_z;
      msg.pos_pts.push_back(pt);
    }

    // Uniform cubic B-spline knot vector. The current 2D spline evaluator mainly
    // uses knot_span, but publishing knots keeps the message close to EGO's Bspline.
    const int n_ctrl = static_cast<int>(result.selected_ctrl_pts.size());
    const int order = std::max(1, result.bspline_order);
    const int knot_count = n_ctrl + order + 1;
    msg.knots.reserve(knot_count);
    for (int i = 0; i < knot_count; ++i) {
      msg.knots.push_back(static_cast<double>(i - order) * result.knot_span);
    }

    bspline_pub_->publish(msg);

    RCLCPP_WARN(
      get_logger(),
      "[BSPLINE_2D] pub traj_id=%d ctrl=%zu order=%d dt=%.3f duration=%.2f",
      msg.traj_id,
      msg.pos_pts.size(),
      msg.order,
      msg.knot_span,
      msg.duration);
  }

  void publishGrid()
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = p_.frame_id;
    msg.info.resolution = static_cast<float>(map_.resolution());
    msg.info.width = static_cast<uint32_t>(map_.width());
    msg.info.height = static_cast<uint32_t>(map_.height());
    msg.info.origin.position.x = map_.origin_x();
    msg.info.origin.position.y = map_.origin_y();
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = map_.data();
    grid_pub_->publish(msg);
  }

  void publishPath(const std::vector<Vec2>& path, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = p_.frame_id;
    msg.poses.reserve(path.size());
    for (const auto& p : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = p_.fixed_z;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    pub->publish(msg);
  }

  void publishLocalGoal(const Vec2& p)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = p_.frame_id;
    msg.pose.position.x = p.x;
    msg.pose.position.y = p.y;
    msg.pose.position.z = p_.fixed_z;
    msg.pose.orientation.w = 1.0;
    local_goal_pub_->publish(msg);
  }

  void publishMarker(const Vec2& p)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = now();
    m.header.frame_id = p_.frame_id;
    m.ns = "ego_2d_planner";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = p_.fixed_z;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.18;
    m.scale.y = 0.18;
    m.scale.z = 0.18;
    m.color.r = 0.1f;
    m.color.g = 1.0f;
    m.color.b = 0.2f;
    m.color.a = 1.0f;
    marker_pub_->publish(m);
  }

private:
  PlannerParams2D p_;
  GridMap2D map_;
  PlannerManager2D manager_;

  FSMExecState exec_state_{INIT};

  bool have_odom_{false};
  bool have_map_{false};
  bool have_goal_{false};
  bool have_new_target_{false};
  bool have_exec_path_{false};

  Vec2 current_pos_;
  double current_z_{0.0};
  Vec2 goal_pos_;

  std::vector<Vec2> selected_path_;
  int plan_fail_count_{0};
  int bspline_traj_id_{0};

  rclcpp::Time last_replan_time_;
  rclcpp::Time emergency_start_time_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_path_pub_;
  rclcpp::Publisher<Bspline2D>::SharedPtr bspline_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoReplanFSM2DNode>());
  rclcpp::shutdown();
  return 0;
}
