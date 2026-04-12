#include <chrono>
#include <cmath>

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

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/px4_iface.hpp"
#include "offboard_core_pkg/math_tool.hpp"
#include "offboard_core_pkg/tasks.hpp"
#include "offboard_core_pkg/tasks/yaw_spin_task.hpp"
#include "offboard_core_pkg/tasks/tsp_grid_task.hpp"



using namespace std::chrono_literals;
using json = nlohmann::json;

class OffboardTest2Node : public rclcpp::Node {
public:
  OffboardTest2Node() : Node("offboard_test2_node")
  {
    // ===== params =====
    takeoff_height_m_= this->declare_parameter<double>("takeoff_height_m", 1.0);
    arrival_error_max_= this->declare_parameter<double>("arrival_error_max", 0.25);
    

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

    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.best_effort();
    qos_pub.durability_volatile();

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_pub);
    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_pub);
    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_pub);
    
    //px4 create
    px4_ = std::make_unique<Px4Iface>(*this,
                                     offboard_control_mode_pub_,
                                     trajectory_setpoint_pub_,
                                     vehicle_command_pub_);

    // ===== subs =====
    rclcpp::QoS qos_sub(rclcpp::KeepLast(10));
    qos_sub.best_effort();

    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos_sub,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg){ ctx_.vehicle_status = *msg; });

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos_sub,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){
        ctx_.local_pos = *msg;

        if (!ctx_.home_inited && ctx_.pos_valid()) {
          ctx_.home_x = ctx_.local_pos.x;
          ctx_.home_y = ctx_.local_pos.y;
          ctx_.home_z = ctx_.local_pos.z;

          ctx_.takeoff_z = static_cast<float>(ctx_.home_z - takeoff_height_m_);
          ctx_.land_z    = ctx_.home_z;

          ctx_.home_inited = true;

          RCLCPP_INFO(get_logger(), "Home set: x=%.2f y=%.2f z=%.2f",
                      ctx_.home_x, ctx_.home_y, ctx_.home_z);
        }
      });
      
    vehicle_land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected",qos_sub,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        ctx_.land_detected = *msg;
      });

    vehicle_att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_sub,
      [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg){
        ctx_.vehicle_att = *msg;
        ctx_.has_attitude = true;

        if (ctx_.home_inited && !ctx_.home_yaw_inited) {
          const float yaw = math_tool::yaw_from_quat(ctx_.vehicle_att.q);
          if (std::isfinite(yaw)) {
            ctx_.home_yaw = yaw - 0.12;
            ctx_.home_yaw_inited = true;
          }
        }
      });

    // ===== 手动写死目标点 =====
std::vector<Eigen::Vector3d> targets = {
    { 1.375,  0.954, -1.0},
    {-0.857,  2.057, -1.0},
    { 0.866,  2.295, -1.0},
    { 1.142,  0.467, -1.0},
    { 0.718, -1.108, -1.0},
    {-0.925, -2.207, -1.0},
};

// 填入 grid
grid_.setPoints(targets);
    // ===== scheduler =====
    build_scheduler();

    timer_ = create_wall_timer(50ms, std::bind(&OffboardTest2Node::on_timer, this));
    last_time_ = now();
  }

private:
  void build_scheduler() {
    sched_.reset();
    sched_.add(std::make_unique<WaitHomeTask>(get_logger()));
    sched_.add(std::make_unique<PresetpointTask>(get_logger()));
    sched_.add(std::make_unique<SetOffboardTask>(get_logger(), *px4_));
    sched_.add(std::make_unique<ArmTask>(get_logger(), *px4_));
    sched_.add(std::make_unique<TakeoffTask>(
    get_logger(), *px4_, arrival_error_max_));
    // sched_.add(std::make_unique<GridStartTask>(get_logger()));
    sched_.add(std::make_unique<offboard_core_pkg::TspGridTask>(
      get_logger(), grid_,
      wp_xy_tol_, wp_z_tol_,
      v_xy_tol_, v_z_tol_, stable_required_,
      dwell_s_
    ));
    sched_.add(std::make_unique<ReturnHomeTask>(get_logger(), return_step_xy_, return_xy_tol_, return_vxy_tol_));
    sched_.add(std::make_unique<HomeStabilizeTask>(get_logger(), home_stabilize_s_));
    sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(get_logger(), *px4_));
  }

  void on_timer() {
    const auto t = now();
    const double dt = (t - last_time_).seconds();
    last_time_ = t;

    if (!sched_.done() && !ctx_.handover_to_px4_land) {
      px4_->publish_offboard_control_mode(ctx_);
    }

    sched_.tick(ctx_, dt);

    if (!sched_.done()) {
      px4_->publish_setpoint_from_ctx(ctx_);
    }

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

  // ===== grid =====
  Grid grid_;

  // ===== publishers =====
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  // ===== subscriptions =====
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_att_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardTest2Node>());
  rclcpp::shutdown();
  return 0;
}