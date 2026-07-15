//仅下视扫盲，用来调试下视扫盲程序

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>
#include <cstdint>
#include <unordered_map>

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
//测试程序：用于测试开局扫盲任务，具体参数调节：// 2. 下视补盲：只开启下视工业相机，不写入 detected_targets
    // sched_.add(std::make_unique<offboard_core_pkg::StartDownBlindCheckTask>(
    //   get_logger(),
    //   4.0,    // timeout_s
    //   0.02,   // align_tol_m
    //   20,     // stable_required，20*50ms=1s
    //   0.45,   // k_img_to_meter，实机需要调
    //   0.12,   // max_step_m
    //   500.0,  // score_thresh，按图形面积调
    //   0.35)); // dup_remove_radius
    // 如果想看逻辑->start_down_blind_check_task.cpp
    // 数据来源：下视工业相机，发布 custom_vision_msgs/msg/servo_target_array，包含目标位置偏差和得分等信息
    // 你也可以直接发布 custom_vision_msgs/msg/servo_target 来单独测试一个目标的补盲逻辑
    // 订阅相机的回调函数看 down_servo_cb，会把数据写入 ctx_.vision_offset 和 ctx_.vision_aligned 等字段，补盲任务会根据这些字段来判断是否对当前 setpoint 进行修正
    // 你可以在 down_servo_cb 里添加一些打印来观察数据
// 1.看飞行的方向是否与纠偏方向一致
// 2.看是否能对准
// 3.可以通过rqt查看视觉部分是否完成识别
class OffboardTest3Node : public rclcpp::Node {
public:
  OffboardTest3Node() : Node("offboard_test3_node")
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

    timeout_s_ = this->declare_parameter<double>("down_blind_timeout_s", 4.0);
    align_tol_m_ = this->declare_parameter<double>("down_blind_align_tol_m", 0.02);
    k_img_to_meter_ = this->declare_parameter<double>("down_blind_k_img_to_meter", 0.45);
    max_step_m_ = this->declare_parameter<double>("down_blind_max_step_m", 0.12);
    // 图像偏差 -> 机体系方向符号
    // dx_img: 图像右为正
    // dy_img: 图像下为正 
    // body_dx: 机体 x 方向修正
    // body_dy: 机体 y 方向修正
    img_to_body_x_sign_ = this->declare_parameter<double>("down_blind_img_to_body_x_sign", -1.0);
    img_to_body_y_sign_ = this->declare_parameter<double>("down_blind_img_to_body_y_sign", 1.0);

    

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
            ctx_.home_yaw = yaw ;
            ctx_.home_yaw_inited = true;
          }
        }
      });
   
    // ===== 地面站命令订阅 =====
    ground_cmd_sub_ = create_subscription<ground_station_msgs::msg::GroundCommand>(
      "/ground_station/cmd_parsed",
      10,
      std::bind(&OffboardTest3Node::ground_cmd_cb, this, std::placeholders::_1));


    // ===== 下视工业相机视觉伺服订阅 =====
    // shape_color_node 输出：/vision/down/servo_targets
    down_servo_sub_ =
      create_subscription<custom_vision_msgs::msg::ServoTargetArray>(
        "/vision/down/servo_targets",
        10,
        std::bind(&OffboardTest3Node::down_servo_cb, this, std::placeholders::_1));

    // ===== scheduler =====
    build_scheduler();

    timer_ = create_wall_timer(50ms, std::bind(&OffboardTest3Node::on_timer, this));
    last_time_ = now();
  }

private:

void down_servo_cb(
  const custom_vision_msgs::msg::ServoTargetArray::SharedPtr msg)
{
  // 下视相机关闭时不接收数据，防止旧帧污染任务。
  if (!ctx_.vision_down_enable) {
    return;
  }

  const uint64_t now_us =
    static_cast<uint64_t>(this->now().nanoseconds() / 1000ULL);

  ctx_.vision_last_update_us = now_us;
  ctx_.vision_down_targets_stamp_us = now_us;
  ctx_.vision_down_targets.clear();

  if (msg->targets.empty()) {
    ctx_.vision_offset = VisionOffset{};
    return;
  }

  // Vision 把稳定 track_id 写在 ServoTarget.x 中。
  const auto read_track_id = [](float raw_id, int32_t& track_id) {
    if (!std::isfinite(raw_id) || raw_id < 0.0f || raw_id > 1000000.0f) {
      return false;
    }

    const float rounded = std::round(raw_id);
    if (std::fabs(raw_id - rounded) > 1e-3f) {
      return false;
    }

    track_id = static_cast<int32_t>(rounded);
    return true;
  };

  // 先统计 ID，重复 ID 整组拒绝，避免错误锁定。
  std::unordered_map<int32_t, int> id_count;

  for (const auto& target : msg->targets) {
    if (target.type == 0 || target.score < 500.0f) {
      continue;
    }

    int32_t track_id = -1;
    if (!read_track_id(target.x, track_id)) {
      continue;
    }

    id_count[track_id]++;
  }

  for (const auto& target : msg->targets) {
    if (target.type == 0 || target.score < 500.0f) {
      continue;
    }

    int32_t track_id = -1;
    if (!read_track_id(target.x, track_id)) {
      continue;
    }

    if (id_count[track_id] != 1) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[VISION_DOWN] reject duplicate track_id=%d",
        track_id);
      continue;
    }

    VisionOffset vo;
    vo.track_id = track_id;
    vo.dx = target.dx;
    vo.dy = target.dy;
    vo.type = target.type;
    vo.score = target.score;
    vo.stamp_us = now_us;

    const float center_cost =
      target.dx * target.dx + target.dy * target.dy;
    const float area_bonus = 1e-6f * target.score;
    vo.cost = center_cost - area_bonus;

    ctx_.vision_down_targets.push_back(vo);
  }

  if (ctx_.vision_down_targets.empty()) {
    ctx_.vision_offset = VisionOffset{};
    return;
  }

  std::sort(
    ctx_.vision_down_targets.begin(),
    ctx_.vision_down_targets.end(),
    [](const VisionOffset& a, const VisionOffset& b) {
      return a.cost < b.cost;
    });

  // 兼容仍读取 vision_offset 的旧逻辑。
  ctx_.vision_offset = ctx_.vision_down_targets.front();

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    500,
    "[VISION_DOWN] targets=%zu best_id=%d type=%d "
    "dx=%.3f dy=%.3f score=%.1f cost=%.4f",
    ctx_.vision_down_targets.size(),
    ctx_.vision_offset.track_id,
    ctx_.vision_offset.type,
    ctx_.vision_offset.dx,
    ctx_.vision_offset.dy,
    ctx_.vision_offset.score,
    ctx_.vision_offset.cost);
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

    RCLCPP_WARN(get_logger(), "[GCS] UNKNOWN cmd_type=%u raw='%s'",
                static_cast<unsigned>(cmd), msg->raw.c_str());
  }


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
    sched_.add(std::make_unique<HoverTask>(get_logger(),2.0));

    // 2. 下视补盲：只开启下视工业相机，不写入 detected_targets
    sched_.add(std::make_unique<offboard_core_pkg::StartDownBlindCheckTask>(
      get_logger(),
      timeout_s_,    // timeout_s
      align_tol_m_,   // align_tol_m
      stable_required_,  // 连续有效视觉帧数
      k_img_to_meter_,   // k_img_to_meter，实机需要调
      max_step_m_,   // max_step_m
      500.0,  // score_thresh，按图形面积调
      0.35, // dup_remove_radius
      img_to_body_x_sign_, // img_to_body_x_sign
      img_to_body_y_sign_)); // img_to_body_y_sign


    sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(
       get_logger(), *px4_));
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

  // ===== parameters =====
  double takeoff_height_m_{1.0};
  double arrival_error_max_{0.25};
  double home_stabilize_s_{1.5};

  double wp_xy_tol_{0.25};
  double wp_z_tol_{0.25};
  double dwell_s_{5.0};
  double timeout_s_{4.0};
  double align_tol_m_{0.02};
  double k_img_to_meter_{0.45};
  double max_step_m_{0.12};
  double img_to_body_x_sign_{-1.0};
  double img_to_body_y_sign_{1.0};

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
  rclcpp::spin(std::make_shared<OffboardTest3Node>());
  rclcpp::shutdown();
  return 0;
}