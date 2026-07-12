// trajectory_test.cpp
// D题立体货架盘点无人机系统 - 固定轨迹测试节点
// 仅24个二维码扫描点触发 FRONT_PRE_ALIGN 中断。
// 过渡点、绕行点、转向点和降落点不触发中断。
//位置坐标：当前飞行的位置坐标是相对于起飞点的坐标，飞机在起飞点起飞就可以了，但是场地参数仍按图纸绝对坐标填写

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/math_tool.hpp"
#include "offboard_core_pkg/px4_iface.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/tasks.hpp"
#include "offboard_core_pkg/tasks/front_pre_align_task.hpp"
#include "offboard_core_pkg/tasks/trajectory_task.hpp"

using namespace std::chrono_literals;

class TrajectoryTestNode : public rclcpp::Node
{
public:
  TrajectoryTestNode()
  : Node("trajectory_test_node")
  {
    // 基础飞行参数
    takeoff_height_m_ = declare_parameter<double>("takeoff_height_m", 1.50);
    arrival_error_max_ = declare_parameter<double>("arrival_error_max", 0.25);
    traj_max_step_m_ = declare_parameter<double>("traj.max_step_m", 0.04);
    traj_arrive_xy_m_ = declare_parameter<double>("traj.arrive_xy_m", 0.12);
    traj_arrive_z_m_ = declare_parameter<double>("traj.arrive_z_m", 0.12);
    scan_hover_s_ = declare_parameter<double>("traj.scan_hover_s", 0.60);
    transit_hover_s_ = declare_parameter<double>("traj.transit_hover_s", 0.0);
    yaw_hold_s_ = declare_parameter<double>("traj.yaw_hold_s", 0.80);

    // 场地坐标参数
    layout_swap_xy_ = declare_parameter<bool>("layout.swap_xy", true);
    layout_x_sign_ = declare_parameter<double>("layout.x_sign", 1.0);
    layout_y_sign_ = declare_parameter<double>("layout.y_sign", 1.0);
    takeoff_center_x_m_ = declare_parameter<double>("layout.takeoff_center_x_m", 0.75);
    takeoff_center_y_m_ = declare_parameter<double>("layout.takeoff_center_y_m", 0.75);
    landing_center_x_m_ = declare_parameter<double>("layout.landing_center_x_m", 4.00);
    landing_center_y_m_ = declare_parameter<double>("layout.landing_center_y_m", 3.00);
    front1_x_m_ = declare_parameter<double>("layout.front1_x_m", 1.50);
    front2_x_m_ = declare_parameter<double>("layout.front2_x_m", 3.50);
    front_y_low_m_ = declare_parameter<double>("layout.front_y_low_m", 1.00);
    front_y_high_m_ = declare_parameter<double>("layout.front_y_high_m", 3.00);
    front_standoff_m_ = declare_parameter<double>("layout.front_standoff_m", 0.55);
    transit_margin_m_ = declare_parameter<double>("layout.transit_margin_m", 0.40);
    qr_top_height_m_ = declare_parameter<double>("layout.qr_top_height_m", 1.40);
    qr_bottom_height_m_ = declare_parameter<double>("layout.qr_bottom_height_m", 1.00);

    // 航向参数：field yaw=0 朝图纸 +x，field yaw=pi 朝图纸 -x
    yaw_face_left_rad_ = declare_parameter<double>("layout.yaw_face_left_rad", 0.0);
    yaw_face_right_rad_ = declare_parameter<double>("layout.yaw_face_right_rad", M_PI);
    yaw_transit_rad_ = declare_parameter<double>("layout.yaw_transit_rad", 0.0);

    // 视觉参数
    enable_front_vision_ = declare_parameter<bool>("vision.front_enable", false);
    enable_down_vision_ = declare_parameter<bool>("vision.down_enable", false);
    front_offset_topic_ = declare_parameter<std::string>(
      "vision.front_offset_topic", "/vision/front/servo_targets");

    ctx_.vision_front_enable = enable_front_vision_;
    ctx_.vision_down_enable = enable_down_vision_;

    // FRONT_PRE_ALIGN 参数
    front_align_timeout_s_ = declare_parameter<double>("front_align.timeout_s", 8.0);
    front_align_tol_m_ = declare_parameter<double>("front_align.align_tol_m", 0.04);
    front_align_stable_required_ = declare_parameter<int>("front_align.stable_required", 12);
    front_align_k_img_to_meter_ = declare_parameter<double>("front_align.k_img_to_meter", 0.35);
    front_align_max_step_m_ = declare_parameter<double>("front_align.max_step_m", 0.08);
    front_align_score_thresh_ = declare_parameter<double>("front_align.score_thresh", 500.0);
    front_align_img_to_body_y_sign_ =
      declare_parameter<double>("front_align.img_to_body_y_sign", 1.0);
    front_align_img_to_body_z_sign_ =
      declare_parameter<double>("front_align.img_to_body_z_sign", 1.0);
    front_align_expected_type_ = declare_parameter<int>("front_align.expected_type", -1);

    // PX4 publishers
    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.best_effort();
    qos_pub.durability_volatile();

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_pub);
    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_pub);
    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_pub);

    vision_front_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>("/vision/front/enable", 10);
    vision_down_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>("/vision/down/enable", 10);

    px4_ = std::make_unique<Px4Iface>(
      *this, offboard_control_mode_pub_, trajectory_setpoint_pub_, vehicle_command_pub_);

    // PX4 subscriptions
    rclcpp::QoS qos_sub(rclcpp::KeepLast(10));
    qos_sub.best_effort();

    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos_sub,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        ctx_.vehicle_status = *msg;
      });

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos_sub,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        ctx_.local_pos = *msg;

        if (!ctx_.home_inited && ctx_.pos_valid()) {
          ctx_.home_x = ctx_.local_pos.x;
          ctx_.home_y = ctx_.local_pos.y;
          ctx_.home_z = ctx_.local_pos.z;
          ctx_.takeoff_z = static_cast<float>(ctx_.home_z - takeoff_height_m_);
          ctx_.land_z = ctx_.home_z;
          ctx_.home_inited = true;

          RCLCPP_INFO(
            get_logger(), "Home set: x=%.2f y=%.2f z=%.2f takeoff_z=%.2f",
            ctx_.home_x, ctx_.home_y, ctx_.home_z, ctx_.takeoff_z);
        }
      });

    vehicle_land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", qos_sub,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        ctx_.land_detected = *msg;
      });

    vehicle_att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_sub,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        ctx_.vehicle_att = *msg;
        ctx_.has_attitude = true;

        const float yaw = math_tool::yaw_from_quat(ctx_.vehicle_att.q);
        if (std::isfinite(yaw)) {
          ctx_.yaw = yaw;
        }

        if (ctx_.home_inited && !ctx_.home_yaw_inited && std::isfinite(yaw)) {
          ctx_.home_yaw = yaw;
          ctx_.home_yaw_inited = true;
        }
      });

    // 前视视觉输入：
    // data[0]=dx, data[1]=dy, data[2]=score, data[3]=type, data[4]=cost(可选)
    front_offset_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      front_offset_topic_, 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (!ctx_.vision_front_enable) {
          return;
        }

        if (msg->data.size() < 4) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[VISION_FRONT] invalid size=%zu, need dx dy score type", msg->data.size());
          return;
        }

        const float dx = msg->data[0];
        const float dy = msg->data[1];
        const float score = msg->data[2];
        const int type = static_cast<int>(std::lround(msg->data[3]));

        if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(score)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "[VISION_FRONT] non-finite input");
          return;
        }

        VisionOffset offset;
        offset.dx = dx;
        offset.dy = dy;
        offset.score = score;
        offset.type = type;
        offset.cost = msg->data.size() >= 5 && std::isfinite(msg->data[4])
          ? msg->data[4] : dx * dx + dy * dy;
        offset.stamp_us = static_cast<uint64_t>(now().nanoseconds() / 1000ULL);
        ctx_.vision_offset = offset;
      });

    build_scheduler();

    timer_ = create_wall_timer(50ms, std::bind(&TrajectoryTestNode::on_timer, this));
    last_time_ = now();

    RCLCPP_WARN(
      get_logger(), "[TRAJ_TEST] ready: front_topic=%s expected_type=%d",
      front_offset_topic_.c_str(), front_align_expected_type_);
  }

private:
  std::pair<double, double> field_to_rel(double field_x_m, double field_y_m) const
  {
    const double dx_field = field_x_m - takeoff_center_x_m_;
    const double dy_field = field_y_m - takeoff_center_y_m_;

    if (layout_swap_xy_) {
      return {layout_x_sign_ * dy_field, layout_y_sign_ * dx_field};
    }

    return {layout_x_sign_ * dx_field, layout_y_sign_ * dy_field};
  }

  double field_yaw_to_local(double field_yaw_rad) const
  {
    const double vx_field = std::cos(field_yaw_rad);
    const double vy_field = std::sin(field_yaw_rad);

    const double vx_local = layout_swap_xy_
      ? layout_x_sign_ * vy_field : layout_x_sign_ * vx_field;
    const double vy_local = layout_swap_xy_
      ? layout_y_sign_ * vx_field : layout_y_sign_ * vy_field;

    return std::atan2(vy_local, vx_local);
  }

  double rel_z_from_height(double target_height_m) const
  {
    return takeoff_height_m_ - target_height_m;
  }

  TrajectoryTask::InputWaypoint make_wp(
    double x_rel, double y_rel, double z_rel,
    bool hover_after, double hover_s,
    bool use_yaw, double yaw,
    bool pre_align_after = false) const
  {
    TrajectoryTask::InputWaypoint wp;
    wp.x = x_rel;
    wp.y = y_rel;
    wp.z = z_rel;
    wp.hover_after = hover_after;
    wp.hover_s = hover_s;
    wp.use_yaw = use_yaw;
    wp.yaw = yaw;
    wp.pre_align_after = pre_align_after;
    return wp;
  }

  void add_transit_field_wp(
    std::vector<TrajectoryTask::InputWaypoint>& wps,
    double field_x_m, double field_y_m, double field_yaw_rad) const
  {
    const auto [x_rel, y_rel] = field_to_rel(field_x_m, field_y_m);
    const double z_rel = rel_z_from_height(takeoff_height_m_);
    const double yaw_local = field_yaw_to_local(field_yaw_rad);

    wps.push_back(make_wp(
      x_rel, y_rel, z_rel, false, transit_hover_s_, true, yaw_local, false));
  }

  void add_yaw_hold_field_wp(
    std::vector<TrajectoryTask::InputWaypoint>& wps,
    double field_x_m, double field_y_m, double target_height_m,
    double field_yaw_rad, double hold_s) const
  {
    const auto [x_rel, y_rel] = field_to_rel(field_x_m, field_y_m);
    const double z_rel = rel_z_from_height(target_height_m);
    const double yaw_local = field_yaw_to_local(field_yaw_rad);

    wps.push_back(make_wp(
      x_rel, y_rel, z_rel, true, hold_s, true, yaw_local, false));
  }

  void add_scan_field_wp(
    std::vector<TrajectoryTask::InputWaypoint>& wps,
    double field_x_m, double field_y_m, double target_height_m,
    double field_yaw_rad) const
  {
    const auto [x_rel, y_rel] = field_to_rel(field_x_m, field_y_m);
    const double z_rel = rel_z_from_height(target_height_m);
    const double yaw_local = field_yaw_to_local(field_yaw_rad);

    // 只有扫描点将 pre_align_after 置为 true。
    wps.push_back(make_wp(
      x_rel, y_rel, z_rel, true, scan_hover_s_, true, yaw_local, true));
  }

  void add_face_snake(
    std::vector<TrajectoryTask::InputWaypoint>& wps,
    const std::string& face_name, double field_x_m,
    double field_yaw_rad, bool top_row_first) const
  {
    const double y0 = front_y_low_m_ + 0.50;
    const double y1 = front_y_low_m_ + 1.00;
    const double y2 = front_y_low_m_ + 1.50;

    RCLCPP_INFO(
      get_logger(), "[ROUTE] face=%s x=%.2f y=(%.2f %.2f %.2f) yaw=%.3f",
      face_name.c_str(), field_x_m, y0, y1, y2, field_yaw_rad);

    if (top_row_first) {
      add_scan_field_wp(wps, field_x_m, y0, qr_top_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y1, qr_top_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y2, qr_top_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y2, qr_bottom_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y1, qr_bottom_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y0, qr_bottom_height_m_, field_yaw_rad);
    } else {
      add_scan_field_wp(wps, field_x_m, y0, qr_bottom_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y1, qr_bottom_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y2, qr_bottom_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y2, qr_top_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y1, qr_top_height_m_, field_yaw_rad);
      add_scan_field_wp(wps, field_x_m, y0, qr_top_height_m_, field_yaw_rad);
    }
  }

  std::vector<TrajectoryTask::InputWaypoint> build_d_task_waypoints() const
  {
    std::vector<TrajectoryTask::InputWaypoint> wps;
    wps.reserve(40);

    const double front1_left_x = front1_x_m_ - front_standoff_m_;
    const double front1_right_x = front1_x_m_ + front_standoff_m_;
    const double front2_left_x = front2_x_m_ - front_standoff_m_;
    const double front2_right_x = front2_x_m_ + front_standoff_m_;
    const double transit_low_y = front_y_low_m_ - transit_margin_m_;

    // A面
    add_transit_field_wp(wps, front1_left_x, transit_low_y, yaw_transit_rad_);
    add_yaw_hold_field_wp(
      wps, front1_left_x, transit_low_y, takeoff_height_m_, yaw_face_left_rad_, yaw_hold_s_);
    add_face_snake(wps, "A", front1_left_x, yaw_face_left_rad_, true);

    // A -> B
    add_transit_field_wp(wps, front1_left_x, transit_low_y, yaw_face_left_rad_);
    add_transit_field_wp(wps, front1_right_x, transit_low_y, yaw_face_left_rad_);
    add_yaw_hold_field_wp(
      wps, front1_right_x, transit_low_y, takeoff_height_m_, yaw_face_right_rad_, yaw_hold_s_);
    add_face_snake(wps, "B", front1_right_x, yaw_face_right_rad_, false);

    // B -> C
    add_transit_field_wp(wps, front2_left_x, front_y_low_m_ + 0.50, yaw_face_right_rad_);
    add_yaw_hold_field_wp(
      wps, front2_left_x, front_y_low_m_ + 0.50,
      takeoff_height_m_, yaw_face_left_rad_, yaw_hold_s_);
    add_face_snake(wps, "C", front2_left_x, yaw_face_left_rad_, true);

    // C -> D
    add_transit_field_wp(wps, front2_left_x, transit_low_y, yaw_face_left_rad_);
    add_transit_field_wp(wps, front2_right_x, transit_low_y, yaw_face_left_rad_);
    add_yaw_hold_field_wp(
      wps, front2_right_x, transit_low_y,
      takeoff_height_m_, yaw_face_right_rad_, yaw_hold_s_);
    add_face_snake(wps, "D", front2_right_x, yaw_face_right_rad_, false);

    // 降落点上方，不触发中断
    add_transit_field_wp(
      wps, landing_center_x_m_, landing_center_y_m_, yaw_face_right_rad_);

    return wps;
  }

  void build_scheduler()
  {
    sched_.clear();

    sched_.add(std::make_unique<WaitHomeTask>(get_logger()));
    sched_.add(std::make_unique<PresetpointTask>(get_logger()));
    sched_.add(std::make_unique<SetOffboardTask>(get_logger(), *px4_));
    sched_.add(std::make_unique<ArmTask>(get_logger(), *px4_));
    sched_.add(std::make_unique<TakeoffTask>(
      get_logger(), *px4_, arrival_error_max_));

    TrajectoryTask::Config traj_cfg;
    traj_cfg.coord_mode = TrajectoryTask::CoordMode::RELATIVE_TO_ENTER;
    traj_cfg.max_step_m = traj_max_step_m_;
    traj_cfg.arrive_xy_m = traj_arrive_xy_m_;
    traj_cfg.arrive_z_m = traj_arrive_z_m_;
    traj_cfg.default_hover_s = scan_hover_s_;
    traj_cfg.enable_tsp_sort = false;
    traj_cfg.waypoints = build_d_task_waypoints();

    const size_t scan_interrupt_count = static_cast<size_t>(std::count_if(
      traj_cfg.waypoints.begin(), traj_cfg.waypoints.end(),
      [](const TrajectoryTask::InputWaypoint& wp) {
        return wp.pre_align_after;
      }));

    RCLCPP_WARN(
      get_logger(),
      "[TRAJ_TEST] route: waypoints=%zu scan_interrupts=%zu takeoff_h=%.2f "
      "scan_hover=%.2f yaw_hold=%.2f max_step=%.3f swap_xy=%d signs=(%.1f %.1f)",
      traj_cfg.waypoints.size(), scan_interrupt_count, takeoff_height_m_,
      scan_hover_s_, yaw_hold_s_, traj_max_step_m_, layout_swap_xy_ ? 1 : 0,
      layout_x_sign_, layout_y_sign_);

    if (scan_interrupt_count != 24) {
      RCLCPP_ERROR(
        get_logger(), "[TRAJ_TEST] expected 24 scan points, got %zu",
        scan_interrupt_count);
    }

    sched_.add(std::make_unique<TrajectoryTask>(get_logger(), traj_cfg));
    sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(
      get_logger(), *px4_));

    sched_.addAux(std::make_unique<offboard_core_pkg::FrontPreAlignTask>(
      get_logger(),
      front_align_timeout_s_,
      front_align_tol_m_,
      front_align_stable_required_,
      front_align_k_img_to_meter_,
      front_align_max_step_m_,
      front_align_score_thresh_,
      front_align_img_to_body_y_sign_,
      front_align_img_to_body_z_sign_,
      front_align_expected_type_));

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

  void handle_front_pre_align_interrupt()
  {
    if (!ctx_.front_pre_align_request) {
      return;
    }

    if (sched_.interrupt("FRONT_PRE_ALIGN", ctx_)) {
      ctx_.front_pre_align_request = false;
      RCLCPP_WARN(get_logger(), "[SCHED] TRAJECTORY -> FRONT_PRE_ALIGN");
      return;
    }

    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[SCHED] failed to interrupt FRONT_PRE_ALIGN");
  }

  void on_timer()
  {
    const auto t = now();
    double dt = (t - last_time_).seconds();
    last_time_ = t;

    if (!std::isfinite(dt) || dt < 0.0) {
      dt = 0.0;
    }
    dt = std::min(dt, 0.20);

    if (!sched_.done() && !ctx_.handover_to_px4_land) {
      px4_->publish_offboard_control_mode(ctx_);
    }

    // 中断检查必须在 sched_.tick() 之前。
    handle_front_pre_align_interrupt();
    sched_.tick(ctx_, dt);
    publish_vision_gates();

    if (!sched_.done() && !ctx_.handover_to_px4_land) {
      px4_->publish_setpoint_from_ctx(ctx_);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[TRAJ_TEST] done=%d valid=%d pos=(%.2f %.2f %.2f) "
      "sp=(%.2f %.2f %.2f) front=%d request=%d aligned=%d",
      sched_.done() ? 1 : 0,
      ctx_.pos_valid() ? 1 : 0,
      ctx_.cx(), ctx_.cy(), ctx_.cz(),
      ctx_.sp_x, ctx_.sp_y, ctx_.sp_z,
      ctx_.vision_front_enable ? 1 : 0,
      ctx_.front_pre_align_request ? 1 : 0,
      ctx_.vision_aligned ? 1 : 0);
  }
private:
  Context ctx_;
  std::unique_ptr<Px4Iface> px4_;
  Scheduler sched_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  double takeoff_height_m_{1.50};
  double arrival_error_max_{0.25};
  double traj_max_step_m_{0.04};
  double traj_arrive_xy_m_{0.12};
  double traj_arrive_z_m_{0.12};
  double scan_hover_s_{0.60};
  double transit_hover_s_{0.0};
  double yaw_hold_s_{0.80};

  bool layout_swap_xy_{true};
  double layout_x_sign_{1.0};
  double layout_y_sign_{1.0};
  double takeoff_center_x_m_{0.75};
  double takeoff_center_y_m_{0.75};
  double landing_center_x_m_{4.00};
  double landing_center_y_m_{3.00};
  double front1_x_m_{1.50};
  double front2_x_m_{3.50};
  double front_y_low_m_{1.00};
  double front_y_high_m_{3.00};
  double front_standoff_m_{0.55};
  double transit_margin_m_{0.20};
  double qr_top_height_m_{1.40};
  double qr_bottom_height_m_{1.00};

  double yaw_face_left_rad_{0.0};
  double yaw_face_right_rad_{M_PI};
  double yaw_transit_rad_{0.0};

  bool enable_front_vision_{false};
  bool enable_down_vision_{false};
  std::string front_offset_topic_{"/vision/front/servo_targets"};

  double front_align_timeout_s_{8.0};
  double front_align_tol_m_{0.04};
  int front_align_stable_required_{12};
  double front_align_k_img_to_meter_{0.35};
  double front_align_max_step_m_{0.08};
  double front_align_score_thresh_{500.0};
  double front_align_img_to_body_y_sign_{1.0};
  double front_align_img_to_body_z_sign_{1.0};
  int front_align_expected_type_{-1};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_front_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_down_enable_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
    vehicle_att_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
    vehicle_land_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
    front_offset_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTestNode>());
  rclcpp::shutdown();
  return 0;
}