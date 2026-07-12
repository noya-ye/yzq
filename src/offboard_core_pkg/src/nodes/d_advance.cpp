// // D_advance.cpp
// // D题指定目标盘点节点
// // 流程：起飞 -> 第一段EGO -> 前视矫正 -> 第二段EGO -> PX4 Land
// // 示例：ros2 run offboard_core_pkg D_advance --ros-args -p target_label:=C5

// #include <algorithm>
// #include <cctype>
// #include <chrono>
// #include <cmath>
// #include <cstdint>
// #include <functional>
// #include <memory>
// #include <stdexcept>
// #include <string>
// #include <utility>

// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "quadrotor_msgs/msg/position_command.hpp"
// #include "rclcpp/qos.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"

// #include "px4_msgs/msg/offboard_control_mode.hpp"
// #include "px4_msgs/msg/trajectory_setpoint.hpp"
// #include "px4_msgs/msg/vehicle_attitude.hpp"
// #include "px4_msgs/msg/vehicle_command.hpp"
// #include "px4_msgs/msg/vehicle_land_detected.hpp"
// #include "px4_msgs/msg/vehicle_local_position.hpp"
// #include "px4_msgs/msg/vehicle_status.hpp"

// #include "offboard_core_pkg/context.hpp"
// #include "offboard_core_pkg/math_tool.hpp"
// #include "offboard_core_pkg/px4_iface.hpp"
// #include "offboard_core_pkg/scheduler.hpp"
// #include "offboard_core_pkg/tasks.hpp"
// #include "offboard_core_pkg/tasks/ego_goto_task.hpp"
// #include "offboard_core_pkg/tasks/front_pre_align_task.hpp"

// using namespace std::chrono_literals;

// class DAdcanceNode : public rclcpp::Node
// {
// public:
//   DAdcanceNode()
//   : Node("D_advance")
//   {
//     target_label_ = normalize_label(declare_parameter<std::string>("target_label", "A1"));
//     validate_label(target_label_);

//     takeoff_height_m_ = declare_parameter<double>("takeoff_height_m", 1.50);
//     arrival_error_max_ = declare_parameter<double>("arrival_error_max", 0.25);

//     layout_swap_xy_ = declare_parameter<bool>("layout.swap_xy", true);
//     layout_x_sign_ = declare_parameter<double>("layout.x_sign", 1.0);
//     layout_y_sign_ = declare_parameter<double>("layout.y_sign", 1.0);
//     takeoff_center_x_m_ = declare_parameter<double>("layout.takeoff_center_x_m", 0.75);
//     takeoff_center_y_m_ = declare_parameter<double>("layout.takeoff_center_y_m", 0.75);
//     landing_center_x_m_ = declare_parameter<double>("layout.landing_center_x_m", 4.00);
//     landing_center_y_m_ = declare_parameter<double>("layout.landing_center_y_m", 3.00);
//     rack1_x_m_ = declare_parameter<double>("layout.rack1_x_m", 1.50);
//     rack2_x_m_ = declare_parameter<double>("layout.rack2_x_m", 3.50);
//     rack_y_low_m_ = declare_parameter<double>("layout.rack_y_low_m", 1.00);
//     rack_standoff_m_ = declare_parameter<double>("layout.rack_standoff_m", 0.55);
//     qr_top_height_m_ = declare_parameter<double>("layout.qr_top_height_m", 1.40);
//     qr_bottom_height_m_ = declare_parameter<double>("layout.qr_bottom_height_m", 1.00);
//     yaw_face_left_rad_ = declare_parameter<double>("layout.yaw_face_left_rad", 0.0);
//     yaw_face_right_rad_ = declare_parameter<double>("layout.yaw_face_right_rad", M_PI);

//     ego_goal_topic_ = declare_parameter<std::string>("ego.goal_topic", "/move_base_simple/goal");
//     ego_goal_frame_ = declare_parameter<std::string>("ego.goal_frame", "camera_init");
//     ego_cmd_topic_ = declare_parameter<std::string>("ego.cmd_topic", "/position_cmd");
//     ego_odom_topic_ = declare_parameter<std::string>("ego.odom_topic", "/fastlio2/lio_odom");

//     ego_cfg_.cmd_timeout_s = declare_parameter<double>("ego.cmd_timeout_s", 0.50);
//     ego_cfg_.odom_timeout_s = declare_parameter<double>("ego.odom_timeout_s", 0.50);
//     ego_cfg_.max_step_m = declare_parameter<double>("ego.max_step_m", 0.03);
//     ego_cfg_.kp_xy = declare_parameter<double>("ego.kp_xy", 0.15);
//     ego_cfg_.kp_z = declare_parameter<double>("ego.kp_z", 0.30);
//     ego_cfg_.max_cmd_xy_m = declare_parameter<double>("ego.max_cmd_xy_m", 0.15);
//     ego_cfg_.max_cmd_z_m = declare_parameter<double>("ego.max_cmd_z_m", 0.08);
//     ego_cfg_.use_velocity_ff = declare_parameter<bool>("ego.use_velocity_ff", false);
//     ego_cfg_.use_acceleration_ff = declare_parameter<bool>("ego.use_acceleration_ff", false);
//     ego_cfg_.vel_ff_scale = declare_parameter<double>("ego.vel_ff_scale", 0.0);
//     ego_cfg_.acc_ff_scale = declare_parameter<double>("ego.acc_ff_scale", 0.0);
//     ego_cfg_.kp_vel_xy = declare_parameter<double>("ego.kp_vel_xy", 0.0);
//     ego_cfg_.kp_vel_z = declare_parameter<double>("ego.kp_vel_z", 0.0);
//     ego_cfg_.max_vel_xy_mps = declare_parameter<double>("ego.max_vel_xy_mps", 0.30);
//     ego_cfg_.max_vel_z_mps = declare_parameter<double>("ego.max_vel_z_mps", 0.20);
//     ego_cfg_.max_acc_xy_mps2 = declare_parameter<double>("ego.max_acc_xy_mps2", 0.40);
//     ego_cfg_.max_acc_z_mps2 = declare_parameter<double>("ego.max_acc_z_mps2", 0.30);
//     ego_cfg_.err_xy_hold_m = declare_parameter<double>("ego.err_xy_hold_m", 2.0);
//     ego_cfg_.err_z_hold_m = declare_parameter<double>("ego.err_z_hold_m", 1.0);
//     ego_cfg_.x_sign = declare_parameter<double>("ego.x_sign", 1.0);
//     ego_cfg_.y_sign = declare_parameter<double>("ego.y_sign", 1.0);
//     ego_cfg_.swap_xy = declare_parameter<bool>("ego.swap_xy", true);
//     ego_cfg_.yaw_align_rad = declare_parameter<double>("ego.yaw_align_rad", 0.0);
//     ego_cfg_.use_ego_yaw = declare_parameter<bool>("ego.use_ego_yaw", false);

//     ego_arrive_xy_m_ = declare_parameter<double>("ego.arrive_xy_m", 0.15);
//     ego_arrive_z_m_ = declare_parameter<double>("ego.arrive_z_m", 0.15);
//     ego_stable_vxy_mps_ = declare_parameter<double>("ego.stable_vxy_mps", 0.15);
//     ego_stable_vz_mps_ = declare_parameter<double>("ego.stable_vz_mps", 0.12);
//     ego_target_stable_s_ = declare_parameter<double>("ego.target_stable_s", 0.40);
//     ego_landing_stable_s_ = declare_parameter<double>("ego.landing_stable_s", 0.50);
//     ego_goal_republish_s_ = declare_parameter<double>("ego.goal_republish_s", 1.0);
//     ego_cmd_guard_s_ = declare_parameter<double>("ego.cmd_guard_s", 0.30);

//     ctx_.vision_front_enable = declare_parameter<bool>("vision.front_enable", false);
//     ctx_.vision_down_enable = declare_parameter<bool>("vision.down_enable", false);
//     front_offset_topic_ = declare_parameter<std::string>(
//       "vision.front_offset_topic", "/vision/front/servo_targets");

//     front_align_timeout_s_ = declare_parameter<double>("front_align.timeout_s", 8.0);
//     front_align_tol_m_ = declare_parameter<double>("front_align.align_tol_m", 0.04);
//     front_align_stable_required_ = declare_parameter<int>("front_align.stable_required", 12);
//     front_align_k_img_to_meter_ =
//       declare_parameter<double>("front_align.k_img_to_meter", 0.35);
//     front_align_max_step_m_ = declare_parameter<double>("front_align.max_step_m", 0.08);
//     front_align_score_thresh_ =
//       declare_parameter<double>("front_align.score_thresh", 500.0);
//     front_align_img_to_body_y_sign_ =
//       declare_parameter<double>("front_align.img_to_body_y_sign", 1.0);
//     front_align_img_to_body_z_sign_ =
//       declare_parameter<double>("front_align.img_to_body_z_sign", 1.0);
//     front_align_expected_type_ = declare_parameter<int>("front_align.expected_type", -1);

//     setup_px4_io();
//     setup_ego_io();
//     setup_vision_io();
//     build_scheduler();

//     timer_ = create_wall_timer(50ms, std::bind(&DAdcanceNode::on_timer, this));
//     last_time_ = now();

//     RCLCPP_WARN(
//       get_logger(), "[D_ADVANCE] ready target=%s flow=EGO1->ALIGN->EGO2->LAND",
//       target_label_.c_str());
//   }

// private:
//   struct TargetPoint
//   {
//     std::string label;
//     double field_x{0.0};
//     double field_y{0.0};
//     double height{0.0};
//     double field_yaw{0.0};
//   };

//   static std::string normalize_label(std::string label)
//   {
//     label.erase(
//       std::remove_if(label.begin(), label.end(),
//         [](unsigned char c) { return std::isspace(c) != 0; }),
//       label.end());

//     if (!label.empty()) {
//       label[0] = static_cast<char>(
//         std::toupper(static_cast<unsigned char>(label[0])));
//     }

//     return label;
//   }

//   static void validate_label(const std::string& label)
//   {
//     const bool valid =
//       label.size() == 2 &&
//       label[0] >= 'A' && label[0] <= 'D' &&
//       label[1] >= '1' && label[1] <= '6';

//     if (!valid) {
//       throw std::invalid_argument(
//         "target_label must be A1~A6, B1~B6, C1~C6 or D1~D6");
//     }
//   }

//   TargetPoint resolve_target(const std::string& label) const
//   {
//     const char face = label[0];
//     const int index = label[1] - '0';
//     const int column = (index - 1) % 3;
//     const bool top_row = index <= 3;
//     const bool rack1 = face == 'A' || face == 'B';
//     const bool left_face = face == 'A' || face == 'C';

//     TargetPoint target;
//     target.label = label;
//     target.field_x =
//       (rack1 ? rack1_x_m_ : rack2_x_m_) +
//       (left_face ? -rack_standoff_m_ : rack_standoff_m_);
//     target.field_y = rack_y_low_m_ + 0.50 * static_cast<double>(column + 1);
//     target.height = top_row ? qr_top_height_m_ : qr_bottom_height_m_;
//     target.field_yaw = left_face ? yaw_face_left_rad_ : yaw_face_right_rad_;
//     return target;
//   }

//   std::pair<double, double> field_to_rel(double field_x, double field_y) const
//   {
//     const double dx = field_x - takeoff_center_x_m_;
//     const double dy = field_y - takeoff_center_y_m_;

//     if (layout_swap_xy_) {
//       return {layout_x_sign_ * dy, layout_y_sign_ * dx};
//     }

//     return {layout_x_sign_ * dx, layout_y_sign_ * dy};
//   }

//   double field_yaw_to_local(double field_yaw) const
//   {
//     const double vx = std::cos(field_yaw);
//     const double vy = std::sin(field_yaw);

//     const double lx =
//       layout_swap_xy_ ? layout_x_sign_ * vy : layout_x_sign_ * vx;
//     const double ly =
//       layout_swap_xy_ ? layout_y_sign_ * vx : layout_y_sign_ * vy;

//     return std::atan2(ly, lx);
//   }

//   offboard_core_pkg::EgoGotoTask::Config make_ego_cfg(
//     const std::string& task_name,
//     const std::string& goal_name,
//     double x_rel,
//     double y_rel,
//     double height_m,
//     double yaw_local,
//     double stable_required_s) const
//   {
//     offboard_core_pkg::EgoGotoTask::Config cfg;
//     cfg.task_name = task_name;
//     cfg.goal_name = goal_name;
//     cfg.goal_frame = ego_goal_frame_;
//     cfg.x_rel = x_rel;
//     cfg.y_rel = y_rel;
//     cfg.height_m = height_m;
//     cfg.yaw_local = yaw_local;
//     cfg.arrive_xy_m = ego_arrive_xy_m_;
//     cfg.arrive_z_m = ego_arrive_z_m_;
//     cfg.stable_vxy_mps = ego_stable_vxy_mps_;
//     cfg.stable_vz_mps = ego_stable_vz_mps_;
//     cfg.stable_required_s = stable_required_s;
//     cfg.goal_republish_s = ego_goal_republish_s_;
//     cfg.cmd_guard_s = ego_cmd_guard_s_;
//     cfg.planner = ego_cfg_;
//     return cfg;
//   }

//   void setup_px4_io()
//   {
//     rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
//     qos_pub.best_effort();
//     qos_pub.durability_volatile();

//     offboard_control_mode_pub_ =
//       create_publisher<px4_msgs::msg::OffboardControlMode>(
//         "/fmu/in/offboard_control_mode", qos_pub);
//     trajectory_setpoint_pub_ =
//       create_publisher<px4_msgs::msg::TrajectorySetpoint>(
//         "/fmu/in/trajectory_setpoint", qos_pub);
//     vehicle_command_pub_ =
//       create_publisher<px4_msgs::msg::VehicleCommand>(
//         "/fmu/in/vehicle_command", qos_pub);

//     px4_ = std::make_unique<Px4Iface>(
//       *this, offboard_control_mode_pub_, trajectory_setpoint_pub_, vehicle_command_pub_);

//     rclcpp::QoS qos_sub(rclcpp::KeepLast(10));
//     qos_sub.best_effort();

//     vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
//       "/fmu/out/vehicle_status_v1", qos_sub,
//       [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
//         ctx_.vehicle_status = *msg;
//       });

//     local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
//       "/fmu/out/vehicle_local_position", qos_sub,
//       [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
//         ctx_.local_pos = *msg;

//         if (!ctx_.home_inited && ctx_.pos_valid()) {
//           ctx_.home_x = ctx_.local_pos.x;
//           ctx_.home_y = ctx_.local_pos.y;
//           ctx_.home_z = ctx_.local_pos.z;
//           ctx_.takeoff_z = static_cast<float>(ctx_.home_z - takeoff_height_m_);
//           ctx_.land_z = ctx_.home_z;
//           ctx_.home_inited = true;

//           RCLCPP_INFO(
//             get_logger(), "Home set: x=%.2f y=%.2f z=%.2f",
//             ctx_.home_x, ctx_.home_y, ctx_.home_z);
//         }
//       });

//     vehicle_att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
//       "/fmu/out/vehicle_attitude", qos_sub,
//       [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
//         ctx_.vehicle_att = *msg;
//         ctx_.has_attitude = true;

//         const float yaw = math_tool::yaw_from_quat(ctx_.vehicle_att.q);
//         if (std::isfinite(yaw)) {
//           ctx_.yaw = yaw;
//         }

//         if (ctx_.home_inited && !ctx_.home_yaw_inited && std::isfinite(yaw)) {
//           ctx_.home_yaw = yaw;
//           ctx_.home_yaw_inited = true;
//         }
//       });

//     vehicle_land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
//       "/fmu/out/vehicle_land_detected", qos_sub,
//       [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
//         ctx_.land_detected = *msg;
//       });
//   }

//   void setup_ego_io()
//   {
//     ego_goal_pub_ =
//       create_publisher<geometry_msgs::msg::PoseStamped>(ego_goal_topic_, 10);

//     ego_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
//       ego_cmd_topic_, rclcpp::SensorDataQoS(),
//       [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
//         ctx_.ego_x = msg->position.x;
//         ctx_.ego_y = msg->position.y;
//         ctx_.ego_z = msg->position.z;
//         ctx_.ego_vx = msg->velocity.x;
//         ctx_.ego_vy = msg->velocity.y;
//         ctx_.ego_vz = msg->velocity.z;
//         ctx_.ego_ax = msg->acceleration.x;
//         ctx_.ego_ay = msg->acceleration.y;
//         ctx_.ego_az = msg->acceleration.z;
//         ctx_.ego_yaw = msg->yaw;
//         ctx_.ego_yaw_dot = msg->yaw_dot;
//         ctx_.ego_traj_id = msg->trajectory_id;
//         ctx_.ego_traj_flag = msg->trajectory_flag;
//         ctx_.ego_cmd_valid = true;
//         ctx_.ego_cmd_stamp_us =
//           static_cast<uint64_t>(now().nanoseconds() / 1000ULL);
//       });

//     ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//       ego_odom_topic_, rclcpp::SensorDataQoS(),
//       [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
//         ctx_.ego_odom_valid = true;
//         ctx_.ego_odom_stamp_us =
//           static_cast<uint64_t>(now().nanoseconds() / 1000ULL);
//         ctx_.ego_odom_x = msg->pose.pose.position.x;
//         ctx_.ego_odom_y = msg->pose.pose.position.y;
//         ctx_.ego_odom_z = msg->pose.pose.position.z;
//         ctx_.ego_odom_vx = msg->twist.twist.linear.x;
//         ctx_.ego_odom_vy = msg->twist.twist.linear.y;
//         ctx_.ego_odom_vz = msg->twist.twist.linear.z;
//       });
//   }

//   void setup_vision_io()
//   {
//     vision_front_enable_pub_ =
//       create_publisher<std_msgs::msg::Bool>("/vision/front/enable", 10);
//     vision_down_enable_pub_ =
//       create_publisher<std_msgs::msg::Bool>("/vision/down/enable", 10);

//     front_offset_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
//       front_offset_topic_, 10,
//       [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
//         if (!ctx_.vision_front_enable || msg->data.size() < 4) {
//           return;
//         }

//         const float dx = msg->data[0];
//         const float dy = msg->data[1];
//         const float score = msg->data[2];
//         const int type = static_cast<int>(std::lround(msg->data[3]));

//         if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(score)) {
//           return;
//         }

//         VisionOffset offset;
//         offset.dx = dx;
//         offset.dy = dy;
//         offset.score = score;
//         offset.type = type;
//         offset.cost = msg->data.size() >= 5 && std::isfinite(msg->data[4])
//           ? msg->data[4] : dx * dx + dy * dy;
//         offset.stamp_us =
//           static_cast<uint64_t>(now().nanoseconds() / 1000ULL);
//         ctx_.vision_offset = offset;
//       });
//   }

//   void build_scheduler()
//   {
//     const TargetPoint target = resolve_target(target_label_);
//     const auto [target_x_rel, target_y_rel] =
//       field_to_rel(target.field_x, target.field_y);
//     const auto [landing_x_rel, landing_y_rel] =
//       field_to_rel(landing_center_x_m_, landing_center_y_m_);
//     const double target_yaw_local = field_yaw_to_local(target.field_yaw);

//     const auto target_cfg = make_ego_cfg(
//       "EGO_TO_TARGET", target.label,
//       target_x_rel, target_y_rel, target.height,
//       target_yaw_local, ego_target_stable_s_);

//     const auto landing_cfg = make_ego_cfg(
//       "EGO_TO_LANDING", "LANDING_POINT",
//       landing_x_rel, landing_y_rel, takeoff_height_m_,
//       target_yaw_local, ego_landing_stable_s_);

//     sched_.clear();
//     sched_.add(std::make_unique<WaitHomeTask>(get_logger()));
//     sched_.add(std::make_unique<PresetpointTask>(get_logger()));
//     sched_.add(std::make_unique<SetOffboardTask>(get_logger(), *px4_));
//     sched_.add(std::make_unique<ArmTask>(get_logger(), *px4_));
//     sched_.add(std::make_unique<TakeoffTask>(
//       get_logger(), *px4_, arrival_error_max_));

//     sched_.add(std::make_unique<offboard_core_pkg::EgoGotoTask>(
//       get_logger(), get_clock(), ego_goal_pub_, target_cfg));

//     sched_.add(std::make_unique<offboard_core_pkg::FrontPreAlignTask>(
//       get_logger(),
//       front_align_timeout_s_,
//       front_align_tol_m_,
//       front_align_stable_required_,
//       front_align_k_img_to_meter_,
//       front_align_max_step_m_,
//       front_align_score_thresh_,
//       front_align_img_to_body_y_sign_,
//       front_align_img_to_body_z_sign_,
//       front_align_expected_type_));

//     sched_.add(std::make_unique<offboard_core_pkg::EgoGotoTask>(
//       get_logger(), get_clock(), ego_goal_pub_, landing_cfg));

//     sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(
//       get_logger(), *px4_));

//     sched_.reset();

//     RCLCPP_WARN(
//       get_logger(),
//       "[D_ADVANCE] target=%s rel=(%.2f %.2f) landing=(%.2f %.2f)",
//       target_label_.c_str(),
//       target_x_rel, target_y_rel,
//       landing_x_rel, landing_y_rel);
//   }

//   void publish_vision_gates()
//   {
//     std_msgs::msg::Bool front_msg;
//     front_msg.data = ctx_.vision_front_enable;
//     vision_front_enable_pub_->publish(front_msg);

//     std_msgs::msg::Bool down_msg;
//     down_msg.data = ctx_.vision_down_enable;
//     vision_down_enable_pub_->publish(down_msg);
//   }

//   void on_timer()
//   {
//     const auto t = now();
//     const double dt = (t - last_time_).seconds();
//     last_time_ = t;

//     if (!sched_.done() && !ctx_.handover_to_px4_land) {
//       px4_->publish_offboard_control_mode(ctx_);
//     }

//     sched_.tick(ctx_, dt);
//     publish_vision_gates();

//     if (!sched_.done() && !ctx_.handover_to_px4_land) {
//       px4_->publish_setpoint_from_ctx(ctx_);
//     }

//     RCLCPP_INFO_THROTTLE(
//       get_logger(), *get_clock(), 2000,
//       "[D_ADVANCE] target=%s done=%d pos=(%.2f %.2f %.2f) "
//       "sp=(%.2f %.2f %.2f) ego=(cmd %d odom %d) front=%d aligned=%d",
//       target_label_.c_str(), sched_.done() ? 1 : 0,
//       ctx_.cx(), ctx_.cy(), ctx_.cz(),
//       ctx_.sp_x, ctx_.sp_y, ctx_.sp_z,
//       ctx_.ego_cmd_valid ? 1 : 0,
//       ctx_.ego_odom_valid ? 1 : 0,
//       ctx_.vision_front_enable ? 1 : 0,
//       ctx_.vision_aligned ? 1 : 0);
//   }

// private:
//   Context ctx_;
//   std::unique_ptr<Px4Iface> px4_;
//   Scheduler sched_;

//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Time last_time_;

//   std::string target_label_{"A1"};

//   double takeoff_height_m_{1.50};
//   double arrival_error_max_{0.25};

//   bool layout_swap_xy_{true};
//   double layout_x_sign_{1.0};
//   double layout_y_sign_{1.0};
//   double takeoff_center_x_m_{0.75};
//   double takeoff_center_y_m_{0.75};
//   double landing_center_x_m_{4.00};
//   double landing_center_y_m_{3.00};
//   double rack1_x_m_{1.50};
//   double rack2_x_m_{3.50};
//   double rack_y_low_m_{1.00};
//   double rack_standoff_m_{0.55};
//   double qr_top_height_m_{1.40};
//   double qr_bottom_height_m_{1.00};
//   double yaw_face_left_rad_{0.0};
//   double yaw_face_right_rad_{M_PI};

//   std::string ego_goal_topic_{"/move_base_simple/goal"};
//   std::string ego_goal_frame_{"camera_init"};
//   std::string ego_cmd_topic_{"/position_cmd"};
//   std::string ego_odom_topic_{"/fastlio2/lio_odom"};

//   offboard_core_pkg::EgoVelPlanner::Config ego_cfg_;
//   double ego_arrive_xy_m_{0.15};
//   double ego_arrive_z_m_{0.15};
//   double ego_stable_vxy_mps_{0.15};
//   double ego_stable_vz_mps_{0.12};
//   double ego_target_stable_s_{0.40};
//   double ego_landing_stable_s_{0.50};
//   double ego_goal_republish_s_{1.0};
//   double ego_cmd_guard_s_{0.30};

//   std::string front_offset_topic_{"/vision/front/servo_targets"};
//   double front_align_timeout_s_{8.0};
//   double front_align_tol_m_{0.04};
//   int front_align_stable_required_{12};
//   double front_align_k_img_to_meter_{0.35};
//   double front_align_max_step_m_{0.08};
//   double front_align_score_thresh_{500.0};
//   double front_align_img_to_body_y_sign_{1.0};
//   double front_align_img_to_body_z_sign_{1.0};
//   int front_align_expected_type_{-1};

//   rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
//     offboard_control_mode_pub_;
//   rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
//     trajectory_setpoint_pub_;
//   rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
//     vehicle_command_pub_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ego_goal_pub_;
//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_front_enable_pub_;
//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_down_enable_pub_;

//   rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
//     vehicle_status_sub_;
//   rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
//     local_pos_sub_;
//   rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
//     vehicle_att_sub_;
//   rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
//     vehicle_land_sub_;
//   rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr
//     ego_cmd_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
//   rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
//     front_offset_sub_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   try {
//     rclcpp::spin(std::make_shared<DAdcanceNode>());
//   } catch (const std::exception& e) {
//     RCLCPP_FATAL(
//       rclcpp::get_logger("D_advance"),
//       "startup failed: %s",
//       e.what());
//   }

//   rclcpp::shutdown();
//   return 0;
// }



// d_advance.cpp
// D题指定货物定向盘点节点
// 固定场地安全路线：起飞 -> 下方安全通道 -> 目标面外侧 -> 指定二维码
//                    -> FRONT_PRE_ALIGN -> 原路退出 -> D面外侧 -> 降落点 -> PX4 Land
//
// 仅使用 TrajectoryTask 位置控制；只有目标二维码点触发 FRONT_PRE_ALIGN。
// 场地参数使用图纸绝对坐标，TrajectoryTask 在进入时转换为相对起飞点坐标。

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
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

class DAdvanceNode : public rclcpp::Node
{
public:
  DAdvanceNode()
  : Node("D_advance")
  {
    target_label_ = normalize_label(declare_parameter<std::string>("target_label", "A1"));
    validate_label(target_label_);

    takeoff_height_m_ = declare_parameter<double>("takeoff_height_m", 1.50);
    arrival_error_max_ = declare_parameter<double>("arrival_error_max", 0.25);
    traj_max_step_m_ = declare_parameter<double>("traj.max_step_m", 0.04);
    traj_arrive_xy_m_ = declare_parameter<double>("traj.arrive_xy_m", 0.12);
    traj_arrive_z_m_ = declare_parameter<double>("traj.arrive_z_m", 0.12);
    target_hover_s_ = declare_parameter<double>("traj.target_hover_s", 0.60);
    transit_hover_s_ = declare_parameter<double>("traj.transit_hover_s", 0.0);
    yaw_hold_s_ = declare_parameter<double>("traj.yaw_hold_s", 0.60);

    layout_swap_xy_ = declare_parameter<bool>("layout.swap_xy", true);
    layout_x_sign_ = declare_parameter<double>("layout.x_sign", 1.0);
    layout_y_sign_ = declare_parameter<double>("layout.y_sign", 1.0);
    takeoff_center_x_m_ = declare_parameter<double>("layout.takeoff_center_x_m", 0.75);
    takeoff_center_y_m_ = declare_parameter<double>("layout.takeoff_center_y_m", 0.75);
    landing_center_x_m_ = declare_parameter<double>("layout.landing_center_x_m", 4.00);
    landing_center_y_m_ = declare_parameter<double>("layout.landing_center_y_m", 3.00);
    rack1_x_m_ = declare_parameter<double>("layout.rack1_x_m", 1.50);
    rack2_x_m_ = declare_parameter<double>("layout.rack2_x_m", 3.50);
    rack_y_low_m_ = declare_parameter<double>("layout.rack_y_low_m", 1.00);
    rack_standoff_m_ = declare_parameter<double>("layout.rack_standoff_m", 0.55);
    qr_top_height_m_ = declare_parameter<double>("layout.qr_top_height_m", 1.40);
    qr_bottom_height_m_ = declare_parameter<double>("layout.qr_bottom_height_m", 1.00);
    yaw_face_left_rad_ = declare_parameter<double>("layout.yaw_face_left_rad", 0.0);
    yaw_face_right_rad_ = declare_parameter<double>("layout.yaw_face_right_rad", M_PI);
    yaw_transit_rad_ = declare_parameter<double>("layout.yaw_transit_rad", 0.0);

    transit_margin_m_ = declare_parameter<double>("route.transit_margin_m", 0.40);
    min_clearance_m_ = declare_parameter<double>("route.min_clearance_m", 0.30);

    enable_front_vision_ = declare_parameter<bool>("vision.front_enable", false);
    enable_down_vision_ = declare_parameter<bool>("vision.down_enable", false);
    front_offset_topic_ = declare_parameter<std::string>(
      "vision.front_offset_topic", "/vision/front/servo_targets");

    ctx_.vision_front_enable = enable_front_vision_;
    ctx_.vision_down_enable = enable_down_vision_;

    front_align_timeout_s_ = declare_parameter<double>("front_align.timeout_s", 8.0);
    front_align_tol_m_ = declare_parameter<double>("front_align.align_tol_m", 0.04);
    front_align_stable_required_ = declare_parameter<int>("front_align.stable_required", 12);
    front_align_k_img_to_meter_ =
      declare_parameter<double>("front_align.k_img_to_meter", 0.35);
    front_align_max_step_m_ = declare_parameter<double>("front_align.max_step_m", 0.08);
    front_align_score_thresh_ =
      declare_parameter<double>("front_align.score_thresh", 500.0);
    front_align_img_to_body_y_sign_ =
      declare_parameter<double>("front_align.img_to_body_y_sign", 1.0);
    front_align_img_to_body_z_sign_ =
      declare_parameter<double>("front_align.img_to_body_z_sign", 1.0);
    front_align_expected_type_ = declare_parameter<int>("front_align.expected_type", -1);

    validate_route_geometry();
    setup_px4_io();
    setup_vision_io();
    build_scheduler();

    timer_ = create_wall_timer(50ms, std::bind(&DAdvanceNode::on_timer, this));
    last_time_ = now();

    const TargetPoint target = resolve_target(target_label_);
    RCLCPP_WARN(
      get_logger(),
      "[D_ADVANCE] ready target=%s field=(%.2f %.2f %.2f) "
      "safe_corridor_y=%.2f flow=TAKEOFF->SAFE_ROUTE->ALIGN->SAFE_EXIT->LAND",
      target.label.c_str(), target.field_x, target.field_y, target.height,
      rack_y_low_m_ - transit_margin_m_);
  }

private:
  struct TargetPoint
  {
    std::string label;
    double field_x{0.0};
    double field_y{0.0};
    double height{0.0};
    double field_yaw{0.0};
  };

  static std::string normalize_label(std::string label)
  {
    label.erase(
      std::remove_if(label.begin(), label.end(),
        [](unsigned char c) { return std::isspace(c) != 0; }),
      label.end());

    if (!label.empty()) {
      label[0] = static_cast<char>(
        std::toupper(static_cast<unsigned char>(label[0])));
    }
    return label;
  }

  static void validate_label(const std::string& label)
  {
    const bool valid =
      label.size() == 2 &&
      label[0] >= 'A' && label[0] <= 'D' &&
      label[1] >= '1' && label[1] <= '6';

    if (!valid) {
      throw std::invalid_argument(
        "target_label must be A1~A6, B1~B6, C1~C6 or D1~D6");
    }
  }

  void validate_route_geometry() const
  {
    if (!std::isfinite(transit_margin_m_) ||
        !std::isfinite(min_clearance_m_) ||
        !std::isfinite(rack_standoff_m_)) {
      throw std::invalid_argument("route clearance parameters must be finite");
    }

    if (min_clearance_m_ <= 0.0) {
      throw std::invalid_argument("route.min_clearance_m must be > 0");
    }

    if (transit_margin_m_ < min_clearance_m_) {
      throw std::invalid_argument(
        "route.transit_margin_m is smaller than route.min_clearance_m");
    }

    if (rack_standoff_m_ < min_clearance_m_) {
      throw std::invalid_argument(
        "layout.rack_standoff_m is smaller than route.min_clearance_m");
    }

    if (rack_y_low_m_ - transit_margin_m_ <= 0.0) {
      throw std::invalid_argument(
        "lower transit corridor is outside the field: rack_y_low - transit_margin <= 0");
    }
  }

  TargetPoint resolve_target(const std::string& label) const
  {
    const char face = label[0];
    const int index = label[1] - '0';
    const int column = (index - 1) % 3;
    const bool top_row = index <= 3;
    const bool rack1 = face == 'A' || face == 'B';
    const bool left_face = face == 'A' || face == 'C';

    TargetPoint target;
    target.label = label;
    target.field_x =
      (rack1 ? rack1_x_m_ : rack2_x_m_) +
      (left_face ? -rack_standoff_m_ : rack_standoff_m_);
    target.field_y = rack_y_low_m_ + 0.50 * static_cast<double>(column + 1);
    target.height = top_row ? qr_top_height_m_ : qr_bottom_height_m_;
    target.field_yaw = left_face ? yaw_face_left_rad_ : yaw_face_right_rad_;
    return target;
  }

  void setup_px4_io()
  {
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

    px4_ = std::make_unique<Px4Iface>(
      *this, offboard_control_mode_pub_, trajectory_setpoint_pub_, vehicle_command_pub_);

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

    vehicle_land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", qos_sub,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        ctx_.land_detected = *msg;
      });
  }

  void setup_vision_io()
  {
    vision_front_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>("/vision/front/enable", 10);
    vision_down_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>("/vision/down/enable", 10);

    front_offset_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      front_offset_topic_, 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (!ctx_.vision_front_enable) {
          return;
        }

        if (msg->data.size() < 4) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[VISION_FRONT] invalid size=%zu, need dx dy score type",
            msg->data.size());
          return;
        }

        const float dx = msg->data[0];
        const float dy = msg->data[1];
        const float score = msg->data[2];
        const int type = static_cast<int>(std::lround(msg->data[3]));

        if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(score)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[VISION_FRONT] non-finite input");
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
  }

  std::pair<double, double> field_to_rel(double field_x, double field_y) const
  {
    const double dx = field_x - takeoff_center_x_m_;
    const double dy = field_y - takeoff_center_y_m_;

    if (layout_swap_xy_) {
      return {layout_x_sign_ * dy, layout_y_sign_ * dx};
    }
    return {layout_x_sign_ * dx, layout_y_sign_ * dy};
  }

  double field_yaw_to_local(double field_yaw) const
  {
    const double vx = std::cos(field_yaw);
    const double vy = std::sin(field_yaw);
    const double lx = layout_swap_xy_ ? layout_x_sign_ * vy : layout_x_sign_ * vx;
    const double ly = layout_swap_xy_ ? layout_y_sign_ * vx : layout_y_sign_ * vy;
    return std::atan2(ly, lx);
  }

  double rel_z_from_height(double target_height) const
  {
    return takeoff_height_m_ - target_height;
  }

  TrajectoryTask::InputWaypoint make_field_wp(
    double field_x, double field_y, double height, double field_yaw,
    bool hover_after = false, double hover_s = 0.0,
    bool pre_align_after = false) const
  {
    const auto [x_rel, y_rel] = field_to_rel(field_x, field_y);

    TrajectoryTask::InputWaypoint wp;
    wp.x = x_rel;
    wp.y = y_rel;
    wp.z = rel_z_from_height(height);
    wp.hover_after = hover_after;
    wp.hover_s = hover_s;
    wp.use_yaw = true;
    wp.yaw = field_yaw_to_local(field_yaw);
    wp.pre_align_after = pre_align_after;
    return wp;
  }

  std::vector<TrajectoryTask::InputWaypoint> build_safe_route() const
  {
    const TargetPoint target = resolve_target(target_label_);
    const double transit_low_y = rack_y_low_m_ - transit_margin_m_;
    const double rack2_right_x = rack2_x_m_ + rack_standoff_m_;

    std::vector<TrajectoryTask::InputWaypoint> wps;
    wps.reserve(10);

    // 1. 先从起飞点竖直投影位置进入货架下方安全通道，避免斜穿货架端点。
    wps.push_back(make_field_wp(
      takeoff_center_x_m_, transit_low_y, takeoff_height_m_, yaw_transit_rad_,
      false, transit_hover_s_, false));

    // 2. 沿安全通道横移到目标面的外侧，不穿过任何货架板。
    wps.push_back(make_field_wp(
      target.field_x, transit_low_y, takeoff_height_m_, yaw_transit_rad_,
      false, transit_hover_s_, false));

    // 3. 在安全通道内先转向并停稳，再接近货架。
    wps.push_back(make_field_wp(
      target.field_x, transit_low_y, takeoff_height_m_, target.field_yaw,
      true, yaw_hold_s_, false));

    // 4. 在安全通道内完成高度调整，避免边靠近货架边大幅升降。
    wps.push_back(make_field_wp(
      target.field_x, transit_low_y, target.height, target.field_yaw,
      false, transit_hover_s_, false));

    // 5. 沿目标面外侧直线接近二维码；仅此点触发 FRONT_PRE_ALIGN。
    wps.push_back(make_field_wp(
      target.field_x, target.field_y, target.height, target.field_yaw,
      true, target_hover_s_, true));

    // 6. 矫正完成后沿原面退回下方安全通道。
    wps.push_back(make_field_wp(
      target.field_x, transit_low_y+2.80, target.height, target.field_yaw,
      false, transit_hover_s_, false));

    // // 7. 在安全通道内恢复巡航高度。
    // wps.push_back(make_field_wp(
    //   target.field_x, transit_low_y, takeoff_height_m_, target.field_yaw,
    //   false, transit_hover_s_, false));

    // // 8. 沿货架下方通道移动到第二个货架的最右侧。
    // wps.push_back(make_field_wp(
    //   rack2_right_x, transit_low_y+2.70, takeoff_height_m_, yaw_transit_rad_,
    //   false, transit_hover_s_, false));

    // 9. 沿D面外侧上行到降落点同一横向位置，避免横穿第二个货架。
    wps.push_back(make_field_wp(
      rack2_right_x, transit_low_y+2.80, takeoff_height_m_, yaw_transit_rad_,
      false, transit_hover_s_, false));

    // 10. 小距离横移到降落圆心上方，再交给 PX4 Land。
    wps.push_back(make_field_wp(
      landing_center_x_m_, landing_center_y_m_, takeoff_height_m_, yaw_transit_rad_,
      false, 0.0, false));

    return wps;
  }

  void build_scheduler()
  {
    TrajectoryTask::Config traj_cfg;
    traj_cfg.coord_mode = TrajectoryTask::CoordMode::RELATIVE_TO_ENTER;
    traj_cfg.max_step_m = traj_max_step_m_;
    traj_cfg.arrive_xy_m = traj_arrive_xy_m_;
    traj_cfg.arrive_z_m = traj_arrive_z_m_;
    traj_cfg.default_hover_s = target_hover_s_;
    traj_cfg.enable_tsp_sort = false;
    traj_cfg.waypoints = build_safe_route();

    const size_t scan_count = static_cast<size_t>(std::count_if(
      traj_cfg.waypoints.begin(), traj_cfg.waypoints.end(),
      [](const TrajectoryTask::InputWaypoint& wp) {
        return wp.pre_align_after;
      }));

    if (scan_count != 1) {
      throw std::runtime_error("D_advance safe route must contain exactly one scan point");
    }

    sched_.clear();
    sched_.add(std::make_unique<WaitHomeTask>(get_logger()));
    sched_.add(std::make_unique<PresetpointTask>(get_logger()));
    sched_.add(std::make_unique<SetOffboardTask>(get_logger(), *px4_));
    sched_.add(std::make_unique<ArmTask>(get_logger(), *px4_));
    sched_.add(std::make_unique<TakeoffTask>(
      get_logger(), *px4_, arrival_error_max_));
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

    RCLCPP_WARN(
      get_logger(),
      "[D_ADVANCE] safe route target=%s waypoints=%zu scan_points=%zu "
      "max_step=%.3f corridor_margin=%.2f standoff=%.2f",
      target_label_.c_str(), traj_cfg.waypoints.size(), scan_count,
      traj_max_step_m_, transit_margin_m_, rack_standoff_m_);
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
      RCLCPP_WARN(
        get_logger(),
        "[D_ADVANCE] target=%s TRAJECTORY -> FRONT_PRE_ALIGN",
        target_label_.c_str());
      return;
    }

    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[D_ADVANCE] failed to interrupt FRONT_PRE_ALIGN");
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

    // 必须先处理中断，再执行当前任务 tick。
    handle_front_pre_align_interrupt();
    sched_.tick(ctx_, dt);
    publish_vision_gates();

    if (!sched_.done() && !ctx_.handover_to_px4_land) {
      px4_->publish_setpoint_from_ctx(ctx_);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[D_ADVANCE] target=%s done=%d valid=%d pos=(%.2f %.2f %.2f) "
      "sp=(%.2f %.2f %.2f) front=%d request=%d aligned=%d",
      target_label_.c_str(), sched_.done() ? 1 : 0,
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

  std::string target_label_{"A1"};

  double takeoff_height_m_{1.50};
  double arrival_error_max_{0.25};
  double traj_max_step_m_{0.04};
  double traj_arrive_xy_m_{0.12};
  double traj_arrive_z_m_{0.12};
  double target_hover_s_{0.60};
  double transit_hover_s_{0.0};
  double yaw_hold_s_{0.60};

  bool layout_swap_xy_{true};
  double layout_x_sign_{1.0};
  double layout_y_sign_{1.0};
  double takeoff_center_x_m_{0.75};
  double takeoff_center_y_m_{0.75};
  double landing_center_x_m_{4.00};
  double landing_center_y_m_{3.00};
  double rack1_x_m_{1.50};
  double rack2_x_m_{3.50};
  double rack_y_low_m_{1.00};
  double rack_standoff_m_{0.55};
  double qr_top_height_m_{1.40};
  double qr_bottom_height_m_{1.00};
  double yaw_face_left_rad_{0.0};
  double yaw_face_right_rad_{M_PI};
  double yaw_transit_rad_{0.0};

  double transit_margin_m_{0.40};
  double min_clearance_m_{0.30};

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

  try {
    rclcpp::spin(std::make_shared<DAdvanceNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("D_advance"),
      "startup failed: %s",
      e.what());
  }

  rclcpp::shutdown();
  return 0;
}