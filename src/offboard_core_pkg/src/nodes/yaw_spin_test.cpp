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

// 自转测试：
// 1. 不开视觉程序，和以前一样看自转是否稳定
// 2. 开视觉程序，看是否能看到目标后停下来，稳定后采样并只加入一个平均目标
// 启动方法：ros2 run offboard_core_pkg yaw_spin_test_node
class YawSpinTestNode : public rclcpp::Node
{
public:
  YawSpinTestNode() : Node("yaw_spin_test_node")
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

    // D435 前视：发现目标后等待稳定时间
    front_stabilize_s_ = this->declare_parameter<double>("front_stabilize_s", 0.5);

    // D435 前视：稳定后采样窗口
    front_sample_s_ = this->declare_parameter<double>("front_sample_s", 0.35);

    // D435 前视：至少采样几帧才认为有效
    front_min_samples_ = this->declare_parameter<int>("front_min_samples", 3);

    // D435 前视：粗定位去重半径。yaw 扫描期间坐标抖动较大，不能用 0.30m 太小阈值
    ctx_.front_duplicate_xy_m = static_cast<float>(
      this->declare_parameter<double>("front_duplicate_xy_m", 1.20));

    // D435 前视：最大目标数量，防止异常炸目标
    ctx_.max_front_targets = this->declare_parameter<int>("max_front_targets", 5);

    // D435 前视：稳定判断速度阈值
    ctx_.front_stable_vxy_thresh = static_cast<float>(
      this->declare_parameter<double>("front_stable_vxy_thresh", 0.08));

    ctx_.front_stable_vz_thresh = static_cast<float>(
      this->declare_parameter<double>("front_stable_vz_thresh", 0.08));

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
        std::bind(&YawSpinTestNode::ground_cmd_cb, this, std::placeholders::_1));

    // ===== 深度相机粗定位目标订阅 =====
    // vision_bridge_node 输出：/vision/front/targets
    front_targets_sub_ =
      create_subscription<custom_vision_msgs::msg::PositionTargetArray>(
        "/vision/front/targets",
        10,
        std::bind(&YawSpinTestNode::front_targets_cb, this, std::placeholders::_1));

    // ===== 不再手动写死目标点，等待 D435 自转扫描 =====
    ctx_.detected_targets.clear();
    ctx_.reset_front_vision_sampling();

    RCLCPP_INFO(
      get_logger(),
      "detected_targets cleared, waiting for D435 yaw scan");

    // ===== scheduler =====
    build_scheduler();

    timer_ = create_wall_timer(50ms, std::bind(&YawSpinTestNode::on_timer, this));
    last_time_ = now();
  }

private:
  uint64_t now_us() const
  {
    return static_cast<uint64_t>(this->now().nanoseconds() / 1000ULL);
  }

  std::vector<VisionPosition> get_valid_front_targets(
    const custom_vision_msgs::msg::PositionTargetArray::SharedPtr msg) const
  {
    std::vector<VisionPosition> out;

    for (const auto& t : msg->targets) {
      const float x = t.x;
      const float y = t.y;
      const float z = t.z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      out.push_back(VisionPosition{x, y, z});
    }

    return out;
  }

  bool select_new_front_target(
    const std::vector<VisionPosition>& targets,
    VisionPosition& out) const
  {
    for (const auto& p : targets) {
      if (!ctx_.is_front_duplicate(p.x, p.y)) {
        out = p;
        return true;
      }
    }

    return false;
  }

  bool select_sample_target_near_active(
    const std::vector<VisionPosition>& targets,
    VisionPosition& out) const
  {
    if (!front_active_candidate_valid_) {
      return false;
    }

    bool found = false;
    float best_d = std::numeric_limits<float>::max();
    VisionPosition best{};

    for (const auto& p : targets) {
      const float dx = p.x - front_active_candidate_.x;
      const float dy = p.y - front_active_candidate_.y;
      const float d = std::sqrt(dx * dx + dy * dy);

      if (d < best_d) {
        best_d = d;
        best = p;
        found = true;
      }
    }

    if (!found) {
      return false;
    }

    if (best_d > front_sample_gate_m_) {
      RCLCPP_INFO(
        get_logger(),
        "[VISION_FRONT] sample target too far from active candidate: d=%.2f gate=%.2f",
        best_d,
        front_sample_gate_m_);

      return false;
    }

    out = best;
    return true;
  }

  bool push_average_front_target()
  {
    if (ctx_.front_sample_pushed) {
      RCLCPP_WARN(get_logger(), "[VISION_FRONT] avg target already pushed in this sample window");
      return false;
    }

    const std::size_t n = ctx_.front_sample_buffer.size();

    if (n < static_cast<std::size_t>(front_min_samples_)) {
      RCLCPP_WARN(
        get_logger(),
        "[VISION_FRONT] sample ignored: only %zu/%d valid samples",
        n,
        front_min_samples_);
      return false;
    }

    float sx = 0.0f;
    float sy = 0.0f;
    float sz = 0.0f;

    for (const auto& p : ctx_.front_sample_buffer) {
      sx += p.x;
      sy += p.y;
      sz += p.z;
    }

    const float inv_n = 1.0f / static_cast<float>(n);

    const float avg_x = sx * inv_n;
    const float avg_y = sy * inv_n;
    const float avg_z = sz * inv_n;

    if (!std::isfinite(avg_x) || !std::isfinite(avg_y) || !std::isfinite(avg_z)) {
      RCLCPP_WARN(get_logger(), "[VISION_FRONT] avg target invalid, ignored");
      return false;
    }

    if (ctx_.is_front_duplicate(avg_x, avg_y)) {
      RCLCPP_WARN(
        get_logger(),
        "[VISION_FRONT] duplicate avg target ignored: x=%.2f y=%.2f z=%.2f samples=%zu total=%zu",
        avg_x, avg_y, avg_z,
        n,
        ctx_.detected_targets.size());

      ctx_.front_sample_pushed = true;
      return false;
    }

    if (ctx_.detected_targets.size() >= static_cast<std::size_t>(ctx_.max_front_targets)) {
      RCLCPP_WARN(
        get_logger(),
        "[VISION_FRONT] max targets reached (%d), ignore avg target x=%.2f y=%.2f z=%.2f",
        ctx_.max_front_targets,
        avg_x, avg_y, avg_z);

      ctx_.front_sample_pushed = true;
      return false;
    }

    ctx_.detected_targets.push_back(VisionPosition{avg_x, avg_y, avg_z});
    ctx_.front_sample_pushed = true;

    RCLCPP_WARN(
      get_logger(),
      "[VISION_FRONT] PUSH AVG target: x=%.2f y=%.2f z=%.2f samples=%zu total=%zu",
      avg_x, avg_y, avg_z,
      n,
      ctx_.detected_targets.size());

    return true;
  }

  void front_targets_cb(
    const custom_vision_msgs::msg::PositionTargetArray::SharedPtr msg)
  {
    // 只有打开前视门控时才处理 D435 目标点
    if (!ctx_.vision_front_enable) {
      return;
    }

    if (msg->targets.empty()) {
      return;
    }

    const auto valid_targets = get_valid_front_targets(msg);
    if (valid_targets.empty()) {
      return;
    }

    // ============================================================
    // IDLE：yaw 正在扫描。
    // 从数组里找第一个“未重复的新目标”。
    // 注意：不能只看数组第一个，因为第一个可能是旧目标。
    // ============================================================
    if (ctx_.front_vision_state == FrontVisionState::IDLE) {
      VisionPosition p;
      if (!select_new_front_target(valid_targets, p)) {
        // RCLCPP_INFO(
        //   get_logger(),
        //   "[VISION_FRONT] all visible targets are duplicate, total=%zu",
        //   ctx_.detected_targets.size());
        return;
      }

      front_active_candidate_ = p;
      front_active_candidate_valid_ = true;

      ctx_.vision_detected = true;
      ctx_.vision_done = false;

      ctx_.front_vision_state = FrontVisionState::WAIT_STABLE;
      ctx_.front_state_stamp_us = now_us();
      ctx_.front_sample_buffer.clear();
      ctx_.front_sample_pushed = false;

      // 先关闭前视，避免等待稳定期间继续刷坐标
      ctx_.vision_front_enable = false;

      RCLCPP_WARN(
        get_logger(),
        "[VISION_FRONT] trigger NEW target: x=%.2f y=%.2f z=%.2f visible=%zu total=%zu -> stop yaw, wait stable %.2fs",
        p.x, p.y, p.z,
        valid_targets.size(),
        ctx_.detected_targets.size(),
        front_stabilize_s_);

      return;
    }

    // ============================================================
    // WAIT_STABLE：等待飞机速度稳定。
    // 这个阶段不接收坐标。
    // ============================================================
    if (ctx_.front_vision_state == FrontVisionState::WAIT_STABLE) {
      return;
    }

    // ============================================================
    // SAMPLING：
    // 从当前数组里选离触发目标最近的点。
    // 不能直接取第一个，因为第一个可能是旧目标。
    // ============================================================
    if (ctx_.front_vision_state == FrontVisionState::SAMPLING) {
      VisionPosition p;
      if (!select_sample_target_near_active(valid_targets, p)) {
        return;
      }

      ctx_.front_sample_buffer.push_back(p);

      // RCLCPP_INFO(
      //   get_logger(),
      //   "[VISION_FRONT] sampling active target: x=%.2f y=%.2f z=%.2f buf=%zu visible=%zu",
      //   p.x, p.y, p.z,
      //   ctx_.front_sample_buffer.size(),
      //   valid_targets.size());

      return;
    }
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
        gx, gy, gz,
        ctx_.detected_targets.size());

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

    // 开局自转：只开启 D435 深度相机门控，扫描粗目标点
    sched_.add(std::make_unique<offboard_core_pkg::YawSpinTask>(
      get_logger(),
      0.5));
    sched_.add(std::make_unique<HoverTask>(
      get_logger(),
      2.0));

    sched_.add(std::make_unique<HomeStabilizeTask>(
      get_logger(),
      home_stabilize_s_));

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

  void publish_task_status()
  {
    ground_station_msgs::msg::TaskStatus msg;
    msg.header.stamp = now();
    msg.task_name = sched_.current_name();
    msg.current_wp = sched_.done() ? sched_.total_count()
                                   : (sched_.current_index() + 1);
    msg.total_wp = sched_.total_count();
    msg.mission_done = sched_.done();

    task_status_pub_->publish(msg);
  }

  void update_front_vision_state_machine(const rclcpp::Time& t)
  {
    // ============================================================
    // WAIT_STABLE：
    // D435 已触发目标，YawSpinTask 应该已经停 yaw。
    // 这里持续关闭前视，等待无人机速度稳定。
    // ============================================================
    if (ctx_.front_vision_state == FrontVisionState::WAIT_STABLE) {
      ctx_.vision_front_enable = false;

      const double wait_s =
        static_cast<double>(now_us() - ctx_.front_state_stamp_us) * 1e-6;

      const bool stable = ctx_.front_pose_stable();

      if (wait_s >= front_stabilize_s_ && stable) {
        ctx_.front_vision_state = FrontVisionState::SAMPLING;
        ctx_.front_state_stamp_us = now_us();
        ctx_.front_sample_buffer.clear();
        ctx_.front_sample_pushed = false;

        // 稳定后才真正重新打开前视，开始采样坐标
        ctx_.vision_front_enable = true;

        RCLCPP_WARN(
          get_logger(),
          "[VISION_FRONT] pose stable -> start sampling %.2fs | v=(%.2f %.2f %.2f)",
          front_sample_s_,
          ctx_.local_pos.vx,
          ctx_.local_pos.vy,
          ctx_.local_pos.vz);
      } else {
        static double last_wait_log_s = -10.0;
        const double now_s = t.seconds();

        if (now_s - last_wait_log_s > 0.5) {
          last_wait_log_s = now_s;

          const float vxy = std::sqrt(
            ctx_.local_pos.vx * ctx_.local_pos.vx +
            ctx_.local_pos.vy * ctx_.local_pos.vy);

          RCLCPP_INFO(
            get_logger(),
            "[VISION_FRONT] wait stable: wait=%.2f/%.2f stable=%d vxy=%.3f vz=%.3f",
            wait_s,
            front_stabilize_s_,
            stable ? 1 : 0,
            vxy,
            std::fabs(ctx_.local_pos.vz));
        }
      }

      return;
    }

    // ============================================================
    // SAMPLING：
    // 此阶段前视保持打开，front_targets_cb() 只缓存坐标。
    // 时间到后平均，只 push 一次，然后通知 YawSpinTask 恢复旋转。
    // ============================================================
    if (ctx_.front_vision_state == FrontVisionState::SAMPLING) {
      ctx_.vision_front_enable = true;

      const double sample_s =
        static_cast<double>(now_us() - ctx_.front_state_stamp_us) * 1e-6;

      if (sample_s >= front_sample_s_) {
        push_average_front_target();

        ctx_.front_sample_buffer.clear();
        ctx_.front_vision_state = FrontVisionState::IDLE;

        front_active_candidate_valid_ = false;

        // 通知 YawSpinTask：稳定采样完成，可以恢复 yaw 扫描
        ctx_.vision_done = true;
        ctx_.vision_detected = false;

        // 交还给 YawSpinTask 继续控制前视门控
        ctx_.vision_front_enable = true;

        RCLCPP_WARN(
          get_logger(),
          "[VISION_FRONT] sampling done %.2fs -> resume yaw scan",
          sample_s);
      }

      return;
    }
  }

  void on_timer()
  {
    const auto t = now();
    const double dt = (t - last_time_).seconds();
    last_time_ = t;

    if (!sched_.done() && !ctx_.handover_to_px4_land) {
      px4_->publish_offboard_control_mode(ctx_);
    }

    // 前视 D435：稳定后采样状态机
    update_front_vision_state_machine(t);

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

  // ===== D435 前视稳定采样控制 =====
  double front_stabilize_s_{0.5};
  double front_sample_s_{0.35};
  int front_min_samples_{3};

  // 当前正在稳定采样的前视候选目标。
  // 用于在 /vision/front/targets 数组里锁定正确目标，
  // 避免数组第一个旧目标干扰新目标采样。
  VisionPosition front_active_candidate_{};
  bool front_active_candidate_valid_{false};
  float front_sample_gate_m_{0.80f};

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
  rclcpp::spin(std::make_shared<YawSpinTestNode>());
  rclcpp::shutdown();
  return 0;
}