//第二次积分赛使用程序：打点+下视扫盲



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

using namespace std::chrono_literals;
using json = nlohmann::json;

// 地面站 + 视觉 + TSP 任务示例
// 最终目标：起飞 -> 自转扫描找目标 -> 视觉锁定细修 -> TSP 路径规划打点 -> 按点飞行 -> 返航降落
// 积分赛最终版本
class OffboardTest2Node : public rclcpp::Node
{
public:
  OffboardTest2Node() : Node("offboard_test2_node")
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
    front_duplicate_xy_m_ = this->declare_parameter<double>("front_duplicate_xy_m", 0.50);
    max_step_m_ = this->declare_parameter<double>("max_step_m", 0.20);
    timeout_s_ = this->declare_parameter<double>("timeout_s", 4.0);
    align_tol_m_ = this->declare_parameter<double>("align_tol_m", 0.02);
    k_img_to_meter_ = this->declare_parameter<double>("k_img_to_meter", 0.45);
    img_to_body_x_sign_ = this->declare_parameter<double>("img_to_body_x_sign", -1.0);
    img_to_body_y_sign_ = this->declare_parameter<double>("img_to_body_y_sign", 1.0);

    // D435 扫描发现目标后，原地停顿采样时间
    // front_stabilize_s_ = this->declare_parameter<double>("front_stabilize_s", 0.5);

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

    task_status_pub_ =
      create_publisher<ground_station_msgs::msg::TaskStatus>(
        "/ground_station/task_status", 10);

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
            ctx_.home_yaw = yaw;
            ctx_.home_yaw_inited = true;
          }
        }
      });

    // ===== 地面站命令订阅 =====
    ground_cmd_sub_ =
      create_subscription<ground_station_msgs::msg::GroundCommand>(
        "/ground_station/cmd_parsed",
        10,
        std::bind(&OffboardTest2Node::ground_cmd_cb, this, std::placeholders::_1));

    // // ===== 深度相机粗定位目标订阅 =====
    // // vision_bridge_node 输出：/vision/front/targets
    // front_targets_sub_ =
    //   create_subscription<custom_vision_msgs::msg::PositionTargetArray>(
    //     "/vision/front/targets",
    //     10,
    //     std::bind(&OffboardTest2Node::front_targets_cb, this, std::placeholders::_1));

    // ===== 下视工业相机视觉伺服订阅 =====
    // shape_color_node 输出：/vision/down/servo_targets
    down_servo_sub_ =
      create_subscription<custom_vision_msgs::msg::ServoTargetArray>(
        "/vision/down/servo_targets",
        10,
        std::bind(&OffboardTest2Node::down_servo_cb, this, std::placeholders::_1));

    // ===== 不再手动写死目标点，等待 D435 自转扫描 =====
    ctx_.detected_targets.clear();

    RCLCPP_INFO(
      get_logger(),
      "detected_targets cleared, waiting for D435 yaw scan");

    // ===== scheduler =====
    build_scheduler();

    timer_ = create_wall_timer(50ms, std::bind(&OffboardTest2Node::on_timer, this));
    last_time_ = now();
  }

private:

void down_servo_cb(
  const custom_vision_msgs::msg::ServoTargetArray::SharedPtr msg)
{
  // 下视相机没开时，不更新偏差，避免旧数据污染
  if (!ctx_.vision_down_enable) {
    return;
  }

  const uint64_t now_us =
    static_cast<uint64_t>(this->now().nanoseconds() / 1000ULL);

  ctx_.vision_last_update_us = now_us;

  // 保存这一帧所有有效下视目标
  ctx_.vision_down_targets.clear();

  if (msg->targets.empty()) {
    ctx_.vision_offset.score = 0.0f;
    ctx_.vision_offset.stamp_us = 0;
    ctx_.vision_down_targets_stamp_us = now_us;
    return;
  }

  for (const auto& t : msg->targets) {
    if (t.type == 0) {
      continue;
    }

    if (t.score < 500.0f) {
      continue;
    }

    VisionOffset vo;
    vo.dx = t.dx;
    vo.dy = t.dy;
    vo.type = t.type;
    vo.score = t.score;
    vo.stamp_us = now_us;

    // cost 越小越优先
    const float center_cost = t.dx * t.dx + t.dy * t.dy;
    const float area_bonus = 1e-6f * t.score;
    vo.cost = center_cost - area_bonus;

    ctx_.vision_down_targets.push_back(vo);
  }

  ctx_.vision_down_targets_stamp_us = now_us;

  if (ctx_.vision_down_targets.empty()) {
    ctx_.vision_offset.score = 0.0f;
    ctx_.vision_offset.stamp_us = 0;
    return;
  }

  // 按 cost 从小到大排序
  std::sort(
    ctx_.vision_down_targets.begin(),
    ctx_.vision_down_targets.end(),
    [](const VisionOffset& a, const VisionOffset& b) {
      return a.cost < b.cost;
    });

  // 兜底：仍然保留第一个最优目标给旧逻辑兼容
  ctx_.vision_offset = ctx_.vision_down_targets.front();

  
}
  void ground_cmd_cb(const ground_station_msgs::msg::GroundCommand::SharedPtr msg)
  {
    const uint8_t cmd = msg->cmd_type;

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_GOTO) {
      if (!msg->has_goal) {
        RCLCPP_WARN(get_logger(), "[GCS] GOTO received but has_goal=false, ignored");
        return;
      }

      const float gx = msg->x;
      const float gy = msg->y;

      // 地面站发送的 z 按“高度”理解：
      // z=1.0 表示离地 1m。
      // PX4 local NED 坐标里，向上是负 z，所以转换成 -1.0。
      const float gz = -std::fabs(msg->z);

      ctx_.detected_targets.push_back(VisionPosition{gx, gy, gz});

      RCLCPP_WARN(
        get_logger(),
        "[GCS] GOTO appended to detected_targets: x=%.2f y=%.2f z=%.2f | total=%zu",
        gx, gy, gz, ctx_.detected_targets.size());

      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_START) {
      RCLCPP_WARN(get_logger(), "[GCS] START received");
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_PAUSE) {
      RCLCPP_WARN(get_logger(), "[GCS] PAUSE received");
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_RESUME) {
      RCLCPP_WARN(get_logger(), "[GCS] RESUME received");
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_RTL) {
      RCLCPP_WARN(get_logger(), "[GCS] RTL received");
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_LAND) {
      RCLCPP_WARN(get_logger(), "[GCS] LAND received");
      ctx_.handover_to_px4_land = true;
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_RESET) {
      RCLCPP_WARN(get_logger(), "[GCS] RESET received");
      return;
    }

    if (cmd == ground_station_msgs::msg::GroundCommand::CMD_CLEAR_TRACK) {
      RCLCPP_WARN(get_logger(), "[GCS] CLEAR_TRACK received");
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "[GCS] UNKNOWN cmd_type=%u raw='%s'",
      static_cast<unsigned>(cmd),
      msg->raw.c_str());
  }
  void add_target(float x, float y, float z)
{
  ctx_.detected_targets.push_back(VisionPosition{x, y, z});

  RCLCPP_WARN(
    get_logger(),
    "[ADD_TARGET] x=%.2f y=%.2f z=%.2f total=%zu",
    x, y, z,
    ctx_.detected_targets.size());
}
 
void build_scheduler()
{
  sched_.reset();

  sched_.add(std::make_unique<WaitHomeTask>(get_logger()));
  sched_.add(std::make_unique<PresetpointTask>(get_logger()));
  sched_.add(std::make_unique<SetOffboardTask>(get_logger(), *px4_));
  sched_.add(std::make_unique<ArmTask>(get_logger(), *px4_));

  sched_.add(std::make_unique<TakeoffTask>(
    get_logger(),
    *px4_,
    arrival_error_max_));

  sched_.add(std::make_unique<HoverTask>(
    get_logger(),
    2.0));

  sched_.add(std::make_unique<offboard_core_pkg::StartDownBlindCheckTask>(
      get_logger(),
      timeout_s_,    // timeout_s
      align_tol_m_,   // align_tol_m
      15,     // stable_required，20*50ms=1s
      k_img_to_meter_,   // k_img_to_meter，实机需要调
      max_step_m_,   // max_step_m
      500.0,  // score_thresh，按图形面积调
      0.35, // dup_remove_radius
      img_to_body_x_sign_, // img_to_body_x_sign
      img_to_body_y_sign_)); // img_to_body_y_sign
  sched_.add(std::make_unique<HoverTask>(
      get_logger(),
      1.0));
  // ============================================================
  // 新逻辑：
  // 不使用 D435，不自转，不走 TSP。
  // 起飞后依次飞四个固定点：
  // ( 1.25,  1.25)
  // ( 1.25, -1.25)
  // (-1.25, -1.25)
  // (-1.25,  1.25)
  // 每到一个点，执行一次 StartDownBlindCheckTask 下视补盲逻辑。
  // ============================================================

  // 1) 飞到 (1.25, 1.25)，然后下视补盲
  sched_.add(std::make_unique<offboard_core_pkg::FixedXYGotoTask>(
    get_logger(),
    0.0f,
    2.15f,
    wp_xy_tol_,
    wp_z_tol_,
    v_xy_tol_,
    stable_required_));

  sched_.add(std::make_unique<offboard_core_pkg::StartDownBlindCheckTask>(
      get_logger(),
      timeout_s_,    // timeout_s
      align_tol_m_,   // align_tol_m
      15,     // stable_required，20*50ms=1s
      k_img_to_meter_,   // k_img_to_meter，实机需要调
      max_step_m_,   // max_step_m
      500.0,  // score_thresh，按图形面积调
      0.35, // dup_remove_radius
      img_to_body_x_sign_, // img_to_body_x_sign
      img_to_body_y_sign_)); // img_to_body_y_sign

  sched_.add(std::make_unique<HoverTask>(
    get_logger(),
    1.0));

  // 2) 飞到 (1.25, -1.25)，然后下视补盲
  sched_.add(std::make_unique<offboard_core_pkg::FixedXYGotoTask>(
    get_logger(),
    -2.0f,
    2.15f,
    wp_xy_tol_,
    wp_z_tol_,
    v_xy_tol_,
    stable_required_));

  sched_.add(std::make_unique<offboard_core_pkg::StartDownBlindCheckTask>(
      get_logger(),
      timeout_s_,    // timeout_s
      align_tol_m_,   // align_tol_m
      15,     // stable_required，20*50ms=1s
      k_img_to_meter_,   // k_img_to_meter，实机需要调
      max_step_m_,   // max_step_m
      500.0,  // score_thresh，按图形面积调
      0.35, // dup_remove_radius
      img_to_body_x_sign_, // img_to_body_x_sign
      img_to_body_y_sign_)); // img_to_body_y_sign

  sched_.add(std::make_unique<HoverTask>(
    get_logger(),
    1.0));

  // 3) 飞到 (-1.25, -1.25)，然后下视补盲
  sched_.add(std::make_unique<offboard_core_pkg::FixedXYGotoTask>(
    get_logger(),
    -2.0f,
    0.0f,
    wp_xy_tol_,
    wp_z_tol_,
    v_xy_tol_,
    stable_required_));

    sched_.add(std::make_unique<offboard_core_pkg::StartDownBlindCheckTask>(
      get_logger(),
      timeout_s_,    // timeout_s
      align_tol_m_,   // align_tol_m
      15,     // stable_required，20*50ms=1s
      k_img_to_meter_,   // k_img_to_meter，实机需要调
      max_step_m_,   // max_step_m
      500.0,  // score_thresh，按图形面积调
      0.35, // dup_remove_radius
      img_to_body_x_sign_, // img_to_body_x_sign
      img_to_body_y_sign_)); // img_to_body_y_sign
  
  sched_.add(std::make_unique<HoverTask>(
    get_logger(),
    1.0));
   

  sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(
    get_logger(),
    *px4_));
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
    case 21: return "illegal_shape";

    default: return "none";
  }
}
  void publish_task_status()
  {
    ground_station_msgs::msg::TaskStatus msg;
    msg.header.stamp = now();
    msg.task_name = sched_.current_name();
    msg.current_wp = sched_.done() ? sched_.total_count()
                                   : (sched_.current_index() + 1);
    msg.total_wp = sched_.total_count();
    msg.mission_done = sched_.done();

    msg.target_type = ctx_.vision_offset.type;//传递当前最佳下视目标的类型，供地面站显示
    msg.target_name = target_type_to_name(ctx_.vision_offset.type);

    task_status_pub_->publish(msg);
  }

  void on_timer()
  {
    const auto t = now();
    const double dt = (t - last_time_).seconds();
    last_time_ = t;

    if (!sched_.done() && !ctx_.handover_to_px4_land) {
      px4_->publish_offboard_control_mode(ctx_);
    }

    sched_.tick(ctx_, dt);

    // 每一拍发布视觉门控
    publish_vision_gates();

    if (!sched_.done() && !ctx_.handover_to_px4_land) {
      px4_->publish_setpoint_from_ctx(ctx_);
    }

    publish_task_status();
  }

private:
  Context ctx_;
  std::unique_ptr<Px4Iface> px4_;
  Scheduler sched_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  // ===== D435 发现目标后暂停采样控制 =====
  bool front_stabilizing_{false};
  rclcpp::Time front_stabilize_start_;
  double front_stabilize_s_{0.5};

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
  double max_step_m_{0.20};

  double return_xy_tol_{0.20};
  double front_duplicate_xy_m_{0.50};
  double return_step_xy_{0.3};
  double return_vxy_tol_{0.15};

  double align_tol_m_{0.02};
  double k_img_to_meter_{0.45};
  double img_to_body_x_sign_{-1.0};
  double img_to_body_y_sign_{1.0};
  double timeout_s_{4.0}; 

  // ===== grid =====
  Grid grid_;

  // ===== publishers =====
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<ground_station_msgs::msg::TaskStatus>::SharedPtr task_status_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_front_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_down_enable_pub_;

  // ===== subscriptions =====
  rclcpp::Subscription<custom_vision_msgs::msg::ServoTarget>::SharedPtr down_selected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_att_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_sub_;
  rclcpp::Subscription<ground_station_msgs::msg::GroundCommand>::SharedPtr ground_cmd_sub_;

  rclcpp::Subscription<custom_vision_msgs::msg::PositionTargetArray>::SharedPtr front_targets_sub_;
  rclcpp::Subscription<custom_vision_msgs::msg::ServoTargetArray>::SharedPtr down_servo_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardTest2Node>());
  rclcpp::shutdown();
  return 0;
}