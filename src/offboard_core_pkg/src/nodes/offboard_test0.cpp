// ego的使用


#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#include <nlohmann/json.hpp>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

#include "ground_station_msgs/msg/task_status.hpp"
#include "ground_station_msgs/msg/ground_command.hpp"

#include "custom_vision_msgs/msg/position_target_array.hpp"
#include "custom_vision_msgs/msg/servo_target_array.hpp"
#include "custom_vision_msgs/msg/servo_target.hpp"


#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/px4_iface.hpp"
#include "offboard_core_pkg/math_tool.hpp"
#include "offboard_core_pkg/tasks.hpp"
#include "offboard_core_pkg/tasks/yaw_spin_task.hpp"
#include "offboard_core_pkg/tasks/tsp_grid_task.hpp"
#include "offboard_core_pkg/tasks/start_down_blind_check_task.hpp"
#include "offboard_core_pkg/tasks/ego_vel_follow_task.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "nav_msgs/msg/odometry.hpp"


using namespace std::chrono_literals;
using json = nlohmann::json;
class OffboardTest0Node : public rclcpp::Node {
public:
  OffboardTest0Node() : Node("offboard_test0_node")
  {
    // ===== params =====
    takeoff_height_m_  = this->declare_parameter<double>("takeoff_height_m", 1.0);
    arrival_error_max_ = this->declare_parameter<double>("arrival_error_max", 0.25);

    wp_xy_tol_ = this->declare_parameter<double>("wp_xy_tol", 0.25);
    wp_z_tol_  = this->declare_parameter<double>("wp_z_tol", 0.25);
    dwell_s_   = this->declare_parameter<double>("dwell_s", 5.0);

    v_xy_tol_ = this->declare_parameter<double>("v_xy_tol", 0.2);
    v_z_tol_  = this->declare_parameter<double>("v_z_tol", 0.2);
    stable_required_ = this->declare_parameter<int>("stable_required", 12);

    return_xy_tol_    = this->declare_parameter<double>("return_xy_tol", 0.20);
    return_step_xy_   = this->declare_parameter<double>("return_step_xy", 0.3);
    return_vxy_tol_   = this->declare_parameter<double>("return_vxy_tol", 0.15);
    home_stabilize_s_ = this->declare_parameter<double>("home_stabilize_s", 1.5);

    // ===== EGO velocity follow params =====
    ego_max_step_m_ = this->declare_parameter<double>(
      "ego.max_step_m", 0.03);

    ego_kp_xy_ = this->declare_parameter<double>(
      "ego.kp_xy", 0.15);
    ego_kp_z_ = this->declare_parameter<double>(
      "ego.kp_z", 0.30);

    ego_max_cmd_xy_m_ = this->declare_parameter<double>(
      "ego.max_cmd_xy_m", 0.15);
    ego_max_cmd_z_m_ = this->declare_parameter<double>(
      "ego.max_cmd_z_m", 0.08);

    ego_use_velocity_ff_ = this->declare_parameter<bool>(
      "ego.use_velocity_ff", true);
    ego_use_acceleration_ff_ = this->declare_parameter<bool>(
      "ego.use_acceleration_ff", true);

    ego_vel_ff_scale_ = this->declare_parameter<double>(
      "ego.vel_ff_scale", 0.0);
    ego_acc_ff_scale_ = this->declare_parameter<double>(
      "ego.acc_ff_scale", 0.0);

    ego_kp_vel_xy_ = this->declare_parameter<double>(
      "ego.kp_vel_xy", 0.0);
    ego_kp_vel_z_ = this->declare_parameter<double>(
      "ego.kp_vel_z", 0.0);

    ego_max_vel_xy_mps_ = this->declare_parameter<double>(
      "ego.max_vel_xy_mps", 0.30);
    ego_max_vel_z_mps_ = this->declare_parameter<double>(
      "ego.max_vel_z_mps", 0.20);

    ego_max_acc_xy_mps2_ = this->declare_parameter<double>(
      "ego.max_acc_xy_mps2", 0.40);
    ego_max_acc_z_mps2_ = this->declare_parameter<double>(
      "ego.max_acc_z_mps2", 0.30);

    ego_err_xy_hold_m_ = this->declare_parameter<double>(
      "ego.err_xy_hold_m", 2.0);
    ego_err_z_hold_m_ = this->declare_parameter<double>(
      "ego.err_z_hold_m", 1.0);

    // 坐标方向参数：调错会直接追反，默认保持你现在验证过的方向
    ego_x_sign_ = this->declare_parameter<double>(
      "ego.x_sign", 1.0);
    ego_y_sign_ = this->declare_parameter<double>(
      "ego.y_sign", 1.0);
    ego_swap_xy_ = this->declare_parameter<bool>(
      "ego.swap_xy", true);
    ego_yaw_align_rad_ = this->declare_parameter<double>(
      "ego.yaw_align_rad", 0.0);

    ego_use_ego_yaw_ = this->declare_parameter<bool>(
      "ego.use_ego_yaw", false);
    

    // ===== pubs =====
    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.best_effort();
    qos_pub.durability_volatile();

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_pub);

    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos_pub);

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_pub);


    // ===== 视觉门控发布 =====
    vision_front_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>("/vision/front/enable", 10);

    vision_down_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>("/vision/down/enable", 10);

    // ===== px4 iface =====
    px4_ = std::make_unique<Px4Iface>(
      *this,
      offboard_control_mode_pub_,
      trajectory_setpoint_pub_,
      vehicle_command_pub_);

    // ===== subs =====
    rclcpp::QoS qos_sub(rclcpp::KeepLast(10));
    qos_sub.best_effort();

    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos_sub,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        ctx_.vehicle_status = *msg;
      });

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos_sub,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        ctx_.local_pos = *msg;

        if (!ctx_.home_inited && ctx_.pos_valid()) {
          ctx_.home_x = ctx_.local_pos.x;
          ctx_.home_y = ctx_.local_pos.y;
          ctx_.home_z = ctx_.local_pos.z;

          ctx_.takeoff_z = static_cast<float>(ctx_.home_z - takeoff_height_m_);
          ctx_.land_z    = ctx_.home_z;

          ctx_.home_inited = true;

          RCLCPP_INFO(
            get_logger(),
            "Home set: x=%.2f y=%.2f z=%.2f",
            ctx_.home_x, ctx_.home_y, ctx_.home_z);
        }
      });

    vehicle_land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", qos_sub,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        ctx_.land_detected = *msg;
      });

    vehicle_att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_sub,
      [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        ctx_.vehicle_att = *msg;
        ctx_.has_attitude = true;

        const float yaw = math_tool::yaw_from_quat(ctx_.vehicle_att.q);
        if (std::isfinite(yaw)) {
          ctx_.yaw = yaw;
        }

        if (ctx_.home_inited && !ctx_.home_yaw_inited) {
          if (std::isfinite(yaw)) {
            ctx_.home_yaw = yaw ;
            ctx_.home_yaw_inited = true;
          }
        }
      });

ego_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
  "/position_cmd",
  rclcpp::SensorDataQoS(),
  [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    ctx_.ego_x = msg->position.x;
    ctx_.ego_y = msg->position.y;
    ctx_.ego_z = msg->position.z;

    ctx_.ego_vx = msg->velocity.x;
    ctx_.ego_vy = msg->velocity.y;
    ctx_.ego_vz = msg->velocity.z;

    ctx_.ego_ax = msg->acceleration.x;
    ctx_.ego_ay = msg->acceleration.y;
    ctx_.ego_az = msg->acceleration.z;

    ctx_.ego_yaw = msg->yaw;
    ctx_.ego_yaw_dot = msg->yaw_dot;
    ctx_.ego_traj_id = msg->trajectory_id;
    ctx_.ego_traj_flag = msg->trajectory_flag;

    ctx_.ego_cmd_valid = true;

    // 这里必须是 ego_cmd_stamp_us，不是 ego_odom_stamp_us
    ctx_.ego_cmd_stamp_us =
      static_cast<uint64_t>(this->now().nanoseconds() / 1000ULL);
  });

ego_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  "/fastlio2/lio_odom",
  rclcpp::SensorDataQoS(),
  [this](const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    ctx_.ego_odom_valid = true;
    ctx_.ego_odom_stamp_us =
      static_cast<uint64_t>(this->now().nanoseconds() / 1000ULL);

    ctx_.ego_odom_x = msg->pose.pose.position.x;
    ctx_.ego_odom_y = msg->pose.pose.position.y;
    ctx_.ego_odom_z = msg->pose.pose.position.z;

    ctx_.ego_odom_vx = msg->twist.twist.linear.x;
    ctx_.ego_odom_vy = msg->twist.twist.linear.y;
    ctx_.ego_odom_vz = msg->twist.twist.linear.z;
  });

    // ===== scheduler =====
    build_scheduler();

    timer_ = create_wall_timer(50ms, std::bind(&OffboardTest0Node::on_timer, this));
    last_time_ = now();
  }

private:
//任务的主要流程由 Scheduler 维护的任务链来执行，build_scheduler() 负责根据当前 Context 构建这个任务链。每当收到新的 GOTO 命令时，都会重建任务链，以确保 GoToTask 能读取到最新的 detected_targets。
  void build_scheduler()
  {
    sched_.clear();

    sched_.add(std::make_unique<WaitHomeTask>(get_logger()));
    sched_.add(std::make_unique<PresetpointTask>(get_logger()));
    sched_.add(std::make_unique<SetOffboardTask>(get_logger(), *px4_));
    sched_.add(std::make_unique<ArmTask>(get_logger(), *px4_));
    sched_.add(std::make_unique<TakeoffTask>(
      get_logger(), *px4_, arrival_error_max_));
    sched_.add(std::make_unique<HoverTask>(
      get_logger(),2.0));
      offboard_core_pkg::EgoVelFollowTask::Config ego_cfg;

      ego_cfg.max_step_m = ego_max_step_m_;

      ego_cfg.kp_xy = ego_kp_xy_;
      ego_cfg.kp_z = ego_kp_z_;

      ego_cfg.max_cmd_xy_m = ego_max_cmd_xy_m_;
      ego_cfg.max_cmd_z_m = ego_max_cmd_z_m_;

      ego_cfg.use_velocity_ff = ego_use_velocity_ff_;
      ego_cfg.use_acceleration_ff = ego_use_acceleration_ff_;

      ego_cfg.vel_ff_scale = ego_vel_ff_scale_;
      ego_cfg.acc_ff_scale = ego_acc_ff_scale_;

      ego_cfg.kp_vel_xy = ego_kp_vel_xy_;
      ego_cfg.kp_vel_z = ego_kp_vel_z_;

      ego_cfg.max_vel_xy_mps = ego_max_vel_xy_mps_;
      ego_cfg.max_vel_z_mps = ego_max_vel_z_mps_;

      ego_cfg.max_acc_xy_mps2 = ego_max_acc_xy_mps2_;
      ego_cfg.max_acc_z_mps2 = ego_max_acc_z_mps2_;

      ego_cfg.err_xy_hold_m = ego_err_xy_hold_m_;
      ego_cfg.err_z_hold_m = ego_err_z_hold_m_;

      // 坐标方向参数：默认保持你原来的方向
      ego_cfg.x_sign = ego_x_sign_;
      ego_cfg.y_sign = ego_y_sign_;
      ego_cfg.swap_xy = ego_swap_xy_;
      ego_cfg.yaw_align_rad = ego_yaw_align_rad_;

      ego_cfg.use_ego_yaw = ego_use_ego_yaw_;

      RCLCPP_WARN(
        get_logger(),
        "[EGO_CFG] max_step=%.3f kp_xy=%.3f kp_z=%.3f max_vel_xy=%.3f max_vel_z=%.3f "
        "max_acc_xy=%.3f max_acc_z=%.3f vel_ff=%d vel_ff_scale=%.2f acc_ff=%d acc_ff_scale=%.2f "
        "x_sign=%.1f y_sign=%.1f swap_xy=%d yaw_align=%.3f use_ego_yaw=%d",
        ego_cfg.max_step_m,
        ego_cfg.kp_xy,
        ego_cfg.kp_z,
        ego_cfg.max_vel_xy_mps,
        ego_cfg.max_vel_z_mps,
        ego_cfg.max_acc_xy_mps2,
        ego_cfg.max_acc_z_mps2,
        ego_cfg.use_velocity_ff ? 1 : 0,
        ego_cfg.vel_ff_scale,
        ego_cfg.use_acceleration_ff ? 1 : 0,
        ego_cfg.acc_ff_scale,
        ego_cfg.x_sign,
        ego_cfg.y_sign,
        ego_cfg.swap_xy ? 1 : 0,
        ego_cfg.yaw_align_rad,
        ego_cfg.use_ego_yaw ? 1 : 0);
      sched_.add(std::make_unique<offboard_core_pkg::EgoVelFollowTask>(
        this->get_logger(),
        ego_cfg));
      sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(
            get_logger(),
            *px4_));
      sched_.reset();
            }

  void publish_vision_gates()
  {
    std_msgs::msg::Bool front_msg;
    front_msg.data = ctx_.vision_front_enable;
    vision_front_enable_pub_->publish(front_msg);

    std_msgs::msg::Bool down_msg;
    down_msg.data = ctx_.vision_down_enable;
    vision_down_enable_pub_->publish(down_msg);
  }
std::string target_type_to_name(int type) const
{
  switch (type) {
    case 1:  return "red_circle";
    case 2:  return "red_square";
    case 3:  return "red_triangle";
    case 4:  return "red_pentagon";
    case 5:  return "red_hexagon";

    case 6:  return "green_circle";
    case 7:  return "green_square";
    case 8:  return "green_triangle";
    case 9:  return "green_pentagon";
    case 10: return "green_hexagon";

    case 11: return "yellow_circle";
    case 12: return "yellow_square";
    case 13: return "yellow_triangle";
    case 14: return "yellow_pentagon";
    case 15: return "yellow_hexagon";

    case 16: return "blue_circle";
    case 17: return "blue_square";
    case 18: return "blue_triangle";
    case 19: return "blue_pentagon";
    case 20: return "blue_hexagon";

    default: return "none";
  }
}
 

void on_timer()
{
  const auto t = now();

  const double dt = std::clamp(
    (t - last_time_).seconds(),
    0.0,
    0.20);

  last_time_ = t;

  // 先让当前任务更新控制模式和 setpoint
  sched_.tick(ctx_, dt);

  // 每一拍发布视觉门控
  publish_vision_gates();

  // Scheduler 已结束或已移交 PX4 LAND，
  // 不再继续发送 Offboard 控制消息
  if (sched_.done() ||
      ctx_.handover_to_px4_land) {
    return;
  }

  // 控制模式与 setpoint 均来自本周期最新 Context
  px4_->publish_offboard_control_mode(ctx_);
  px4_->publish_setpoint_from_ctx(ctx_);
}

private:
  Context ctx_;
  std::unique_ptr<Px4Iface> px4_;
  Scheduler sched_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  // ===== parameters =====
  double takeoff_height_m_{1.0};
  double arrival_error_max_{0.25};
  double home_stabilize_s_{1.5};

  double wp_xy_tol_{0.25};
  double wp_z_tol_{0.25};
  double dwell_s_{5.0};
  

  double v_xy_tol_{0.2};
  double v_z_tol_{0.2};
  int    stable_required_{12};

  double return_xy_tol_{0.20};
  double return_step_xy_{0.3};
  double return_vxy_tol_{0.15};
  // ===== EGO velocity follow parameters =====
  double ego_max_step_m_{0.03};

  double ego_kp_xy_{0.15};
  double ego_kp_z_{0.30};

  double ego_max_cmd_xy_m_{0.15};
  double ego_max_cmd_z_m_{0.08};

  bool ego_use_velocity_ff_{false};
  bool ego_use_acceleration_ff_{false};

  double ego_vel_ff_scale_{0.0};
  double ego_acc_ff_scale_{0.0};

  double ego_kp_vel_xy_{0.0};
  double ego_kp_vel_z_{0.0};

  double ego_max_vel_xy_mps_{0.30};
  double ego_max_vel_z_mps_{0.20};

  double ego_max_acc_xy_mps2_{0.40};
  double ego_max_acc_z_mps2_{0.30};

  double ego_err_xy_hold_m_{2.0};
  double ego_err_z_hold_m_{1.0};

  double ego_x_sign_{1.0};
  double ego_y_sign_{1.0};
  bool ego_swap_xy_{true};
  double ego_yaw_align_rad_{0.0};

  bool ego_use_ego_yaw_{false};


  // ===== grid =====
  Grid grid_;

  // ===== publishers =====
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_front_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_down_enable_pub_;

  // ===== subscriptions =====
 
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_att_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_sub_;


  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr ego_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardTest0Node>());
  rclcpp::shutdown();
  return 0;
}
