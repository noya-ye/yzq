// #include <chrono>
// #include <cmath>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp/qos.hpp"

// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "std_msgs/msg/float32.hpp"

// #include <nlohmann/json.hpp>

// #include "px4_msgs/msg/offboard_control_mode.hpp"
// #include "px4_msgs/msg/trajectory_setpoint.hpp"
// #include "px4_msgs/msg/vehicle_command.hpp"
// #include "px4_msgs/msg/vehicle_local_position.hpp"
// #include "px4_msgs/msg/vehicle_status.hpp"
// #include "px4_msgs/msg/vehicle_attitude.hpp"

// #include "offboard_bb_pkg/context.hpp"
// #include "offboard_bb_pkg/scheduler.hpp"
// #include "offboard_bb_pkg/px4_iface.hpp"
// #include "offboard_bb_pkg/grid.hpp"
// #include "offboard_bb_pkg/tasks.hpp"

// using namespace std::chrono_literals;
// using json = nlohmann::json;

// class OffboardBBNode : public rclcpp::Node {
// public:
//   OffboardBBNode() : Node("offboard_mission1_node")
//   {
//     // ===== params =====
//     ctx_.rows      = this->declare_parameter<int>("grid_rows", 5);
//     ctx_.cols      = this->declare_parameter<int>("grid_cols", 5);
//     ctx_.cell_size = this->declare_parameter<double>("cell_size", 0.8);
//     ctx_.origin_dx = this->declare_parameter<double>("origin_dx", 0.8);
//     ctx_.origin_dy = this->declare_parameter<double>("origin_dy", 0.8);

//     takeoff_height_m_ = this->declare_parameter<double>("takeoff_height_m", 1.0);
//     takeoff_vz_mps_   = this->declare_parameter<double>("takeoff_vz_mps", 0.3);

//     wp_xy_tol_ = this->declare_parameter<double>("wp_xy_tol", 0.25);
//     wp_z_tol_  = this->declare_parameter<double>("wp_z_tol", 0.25);
//     dwell_s_   = this->declare_parameter<double>("dwell_s", 5.0);

//     v_xy_tol_ = this->declare_parameter<double>("v_xy_tol", 0.2);
//     v_z_tol_  = this->declare_parameter<double>("v_z_tol", 0.2);
//     stable_required_ = this->declare_parameter<int>("stable_required", 12);

//     return_xy_tol_    = this->declare_parameter<double>("return_xy_tol", 0.20);
//     return_step_xy_   = this->declare_parameter<double>("return_step_xy", 0.3);
//     return_vxy_tol_   = this->declare_parameter<double>("return_vxy_tol", 0.15);
//     home_stabilize_s_ = this->declare_parameter<double>("home_stabilize_s", 1.5);

//     // ===== pubs QoS =====
//     rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
//     qos_pub.best_effort();
//     qos_pub.durability_volatile();

//     offboard_control_mode_pub_ =
//       create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_pub);
//     trajectory_setpoint_pub_ =
//       create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_pub);
//     vehicle_command_pub_ =
//       create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_pub);

//     // /vision/enable
//     rclcpp::QoS qos_enable(rclcpp::KeepLast(1));
//     qos_enable.reliable();
//     qos_enable.transient_local();
//     vision_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/enable", qos_enable);

//     px4_ = std::make_unique<Px4Iface>(*this,
//                                      offboard_control_mode_pub_,
//                                      trajectory_setpoint_pub_,
//                                      vehicle_command_pub_);

//     // ===== subs =====
//     rclcpp::QoS qos_sub(rclcpp::KeepLast(10));
//     qos_sub.best_effort();

//     vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
//       "/fmu/out/vehicle_status", qos_sub,
//       [this](px4_msgs::msg::VehicleStatus::SharedPtr msg){ ctx_.vehicle_status = *msg; });

//     local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
//       "/fmu/out/vehicle_local_position", qos_sub,
//       [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){
//         ctx_.local_pos = *msg;

//         if (!ctx_.home_inited && ctx_.pos_valid()) {
//           ctx_.home_x = ctx_.local_pos.x;
//           ctx_.home_y = ctx_.local_pos.y;
//           ctx_.home_z = ctx_.local_pos.z;

//           ctx_.takeoff_z = static_cast<float>(ctx_.home_z - takeoff_height_m_);
//           ctx_.land_z    = ctx_.home_z;

//           grid_.build(ctx_);
//           ctx_.home_inited = true;

//           RCLCPP_INFO(get_logger(), "Home set: x=%.2f y=%.2f z=%.2f",
//                       ctx_.home_x, ctx_.home_y, ctx_.home_z);
//         }
//       });

//     vehicle_att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
//       "/fmu/out/vehicle_attitude", qos_sub,
//       [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg){
//         ctx_.vehicle_att = *msg;
//         ctx_.has_attitude = true;

//         if (ctx_.home_inited && !ctx_.home_yaw_inited) {
//           const float yaw = yaw_from_quat(ctx_.vehicle_att.q);
//           if (std::isfinite(yaw)) {
//             ctx_.home_yaw = yaw - 0.12;
//             ctx_.home_yaw_inited = true;
//           }
//         }
//       });

//     // ===== vision detections =====
//     vision_sub_ = create_subscription<std_msgs::msg::String>(
//       "/vision/detections", 10,
//       [this](const std_msgs::msg::String::SharedPtr msg){
//         try {
//           auto j = json::parse(msg->data);

//           uint64_t stamp_ns = 0;
//           if (j.contains("stamp_ns")) stamp_ns = j["stamp_ns"].get<uint64_t>();
//           const uint64_t stamp_us = stamp_ns / 1000ULL;

//           BlockType best_type = BlockType::NONE;
//           float best_score = 0.0f;

//           if (j.contains("detections") && j["detections"].is_array()) {
//             for (auto &d : j["detections"]) {
//               const std::string color = d.value("color", "");
//               const std::string shape = d.value("shape", "");
//               const float score = static_cast<float>(d.value("score", 0.0));

//               if (color == "yellow" && shape == "hexagon") {
//                 if (score > best_score) { best_score = score; best_type = BlockType::YELLOW_HEX; }
//               } else if (color == "blue" && shape == "rectangle") {
//                 if (score > best_score) { best_score = score; best_type = BlockType::BLUE_RECT; }
//               } else if (color == "red" && shape == "circle") {
//                 if (score > best_score) { best_score = score; best_type = BlockType::RED_CIRCLE; }
//               }
//             }
//           }

//           ctx_.vision.type = best_type;
//           ctx_.vision.score = best_score;
//           ctx_.vision.stamp_us = stamp_us;

//         } catch (const std::exception &) {}
//       }
//     );

//     // ===== scheduler =====
//     build_scheduler();

//     timer_ = create_wall_timer(50ms, std::bind(&OffboardBBNode::on_timer, this));
//     last_time_ = now();
//   }

// private:
//   void build_scheduler() {
//     sched_.reset();
//     sched_.add(std::make_unique<WaitHomeTask>(get_logger()));
//     sched_.add(std::make_unique<PresetpointTask>(get_logger()));
//     sched_.add(std::make_unique<SetOffboardTask>(get_logger(), *px4_));
//     sched_.add(std::make_unique<ArmTask>(get_logger(), *px4_));
//     sched_.add(std::make_unique<TakeoffTask>(get_logger(), takeoff_vz_mps_, wp_z_tol_));
//     sched_.add(std::make_unique<GridStartTask>(get_logger()));
//     sched_.add(std::make_unique<GridTraverseTask>(
//       get_logger(), grid_,
//       wp_xy_tol_, wp_z_tol_,
//       v_xy_tol_, v_z_tol_, stable_required_,
//       dwell_s_,
//       0.5));
//     sched_.add(std::make_unique<ReturnHomeTask>(get_logger(), return_step_xy_, return_xy_tol_, return_vxy_tol_));
//     sched_.add(std::make_unique<HomeStabilizeTask>(get_logger(), home_stabilize_s_));
//     sched_.add(std::make_unique<LandDescendTask>(get_logger()));
//     sched_.add(std::make_unique<DisarmTask>(get_logger(), *px4_));
//   }

//   void on_timer() {
//     const auto t = now();
//     const double dt = (t - last_time_).seconds();
//     last_time_ = t;

//     if (!sched_.done()) {
//       px4_->publish_offboard_control_mode(ctx_);
//     }

//     sched_.tick(ctx_, dt);

//     if (!sched_.done()) {
//       px4_->publish_setpoint_from_ctx(ctx_);
//     }

//     // vision enable 周期发布
//     publish_enable_accum_ += dt;
//     if (publish_enable_accum_ >= 0.2) {
//       publish_enable_accum_ = 0.0;
//       std_msgs::msg::Bool m;
//       m.data = ctx_.vision_enable;
//       vision_enable_pub_->publish(m);
//       last_vision_enable_ = ctx_.vision_enable;
//     }
//   }

// private:
//   // publishers / subscribers / context / scheduler ...
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<OffboardBBNode>());
//   rclcpp::shutdown();
//   return 0;
// }