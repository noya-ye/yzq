// 25年国赛：蛇形遍历 + A* 绕障 + 下视补盲中断

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <cctype>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "ground_station_msgs/msg/ground_command.hpp"
#include "ground_station_msgs/msg/mission_plan.hpp"
#include "ground_station_msgs/msg/task_status.hpp"
#include "ground_station_msgs/msg/animal_info.hpp"
#include "ground_station_msgs/msg/mission_result.hpp"

#include "custom_vision_msgs/msg/servo_target_array.hpp"

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/math_tool.hpp"
#include "offboard_core_pkg/px4_iface.hpp"
#include "offboard_core_pkg/scheduler.hpp"
#include "offboard_core_pkg/tasks.hpp"
#include "offboard_core_pkg/tasks/snake_grid_task.hpp"
#include "offboard_core_pkg/tasks/start_down_blind_check_task.hpp"
#include "offboard_core_pkg/planners/a_star_goto_task.hpp"
#include "offboard_core_pkg/tasks/land_45_task.hpp"

using namespace std::chrono_literals;

class SnakeTest : public rclcpp::Node
{
public:
  SnakeTest()
  : Node("snake_test")
  {
    declareParameters();
    createPublishers();
    createSubscribers();

    px4_ = std::make_unique<Px4Iface>(
      *this,
      offboard_control_mode_pub_,
      trajectory_setpoint_pub_,
      vehicle_command_pub_);

    buildScheduler();

    timer_ = create_wall_timer(
      50ms,
      std::bind(&SnakeTest::onTimer, this));

    last_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "[SNAKE_TEST] ready: grid=%dx%d cell=%.2f obstacles=%zu",
      snake_x_cells_,
      snake_y_cells_,
      snake_cell_size_,
      obstacle_cells_.size());
  }

private:
  void declareParameters()
  {
    takeoff_height_m_ =
      declare_parameter<double>("takeoff_height_m", 1.0);

    arrival_error_max_ =
      declare_parameter<double>("arrival_error_max", 0.25);

    // ===== 蛇形网格参数 =====
    snake_x_cells_ =
      declare_parameter<int>("snake_x_cells", 9);

    snake_y_cells_ =
      declare_parameter<int>("snake_y_cells", 7);

    snake_cell_size_ =
      declare_parameter<double>("snake_cell_size", 0.5);

    snake_hover_s_ =
      declare_parameter<double>("snake_hover_s", 0.5);

    snake_max_step_m_ =
      declare_parameter<double>("snake_max_step_m", 0.03);

    snake_arrive_xy_m_ =
      declare_parameter<double>("snake_arrive_xy_m", 0.12);

    snake_arrive_z_m_ =
      declare_parameter<double>("snake_arrive_z_m", 0.12);

    // ===== 下视补盲参数 =====
    timeout_s_ =
      declare_parameter<double>("timeout_s", 4.0);

    align_tol_m_ =
      declare_parameter<double>("align_tol_m", 0.02);

    k_img_to_meter_ =
      declare_parameter<double>("k_img_to_meter", 0.45);

    max_step_m_ =
      declare_parameter<double>("max_step_m", 0.20);

    img_to_body_x_sign_ =
      declare_parameter<double>("img_to_body_x_sign", -1.0);

    img_to_body_y_sign_ =
      declare_parameter<double>("img_to_body_y_sign", 1.0);

    const auto raw_obstacles =
      declare_parameter<std::vector<int64_t>>(
        "obstacle_cells",
        std::vector<int64_t>{});

    if (!parseObstacleCells(raw_obstacles)) {
      throw std::invalid_argument(
        "invalid obstacle_cells parameter");
    }
  }

  bool parseObstacleCells(
    const std::vector<int64_t>& values)
  {
    obstacle_cells_.clear();

    if (values.size() % 2 != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "[SNAKE] obstacle_cells must contain ix/iy pairs");

      return false;
    }

    for (std::size_t i = 0; i < values.size(); i += 2) {
      const int ix = static_cast<int>(values[i]);
      const int iy = static_cast<int>(values[i + 1]);

      if (ix < 0 || ix >= snake_x_cells_ ||
          iy < 0 || iy >= snake_y_cells_) {
        RCLCPP_ERROR(
          get_logger(),
          "[SNAKE] obstacle outside grid: (r=%d,c=%d)",
          iy,
          ix);

        return false;
      }

      if (ix == 0 && iy == 0) {
        RCLCPP_ERROR(
          get_logger(),
          "[SNAKE] start cell cannot be an obstacle");

        return false;
      }

      const bool duplicate = std::any_of(
        obstacle_cells_.begin(),
        obstacle_cells_.end(),
        [ix, iy](const SnakeGridTask::ObstacleCell& cell) {
          return cell.ix == ix && cell.iy == iy;
        });

      if (!duplicate) {
        obstacle_cells_.push_back(
          SnakeGridTask::ObstacleCell{ix, iy});
      }
    }//判断障碍是否合法，并把障碍转化为snake可以读懂的形式

    return true;
  }

  void createPublishers()
  {
    rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
    px4_qos.best_effort();
    px4_qos.durability_volatile();

    landing_led_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/landing/led_enable",
      10);

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode",
        px4_qos);

    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint",
        px4_qos);

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command",
        px4_qos);

    task_status_pub_ =
      create_publisher<ground_station_msgs::msg::TaskStatus>(
        "/ground_station/task_status",
        10);

    mission_plan_pub_ =
      create_publisher<ground_station_msgs::msg::MissionPlan>(
        "/ground_station/mission_plan",
        1);
    animal_info_pub_ =
      create_publisher<ground_station_msgs::msg::AnimalInfo>(
        "/ground_station/animal_info",
        10);//animal_info_pub_,用于巡查过程中实时上报一次动物识别结



    mission_result_pub_ =
      create_publisher<ground_station_msgs::msg::MissionResult>(
        "/ground_station/mission_result",
        1);
    vision_front_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(
        "/vision/front/enable",
        10);

    vision_down_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(
        "/vision/down/enable",
        10);
  }

  void createSubscribers()
  {
    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();

    vehicle_status_sub_ =
      create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        px4_qos,
        [this](
          const px4_msgs::msg::VehicleStatus::SharedPtr msg)
        {
          ctx_.vehicle_status = *msg;
        });

    local_pos_sub_ =
      create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        px4_qos,
        [this](
          const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
        {
          ctx_.local_pos = *msg;

          if (!ctx_.home_inited && ctx_.pos_valid()) {
            ctx_.home_x = ctx_.local_pos.x;
            ctx_.home_y = ctx_.local_pos.y;
            ctx_.home_z = ctx_.local_pos.z;

            ctx_.takeoff_z = static_cast<float>(
              ctx_.home_z - takeoff_height_m_);

            ctx_.land_z = ctx_.home_z;
            ctx_.home_inited = true;

            RCLCPP_INFO(
              get_logger(),
              "Home set: x=%.2f y=%.2f z=%.2f",
              ctx_.home_x,
              ctx_.home_y,
              ctx_.home_z);
          }
        });

    vehicle_att_sub_ =
      create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude",
        px4_qos,
        [this](
          const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
        {
          ctx_.vehicle_att = *msg;
          ctx_.has_attitude = true;

          const float yaw =
            math_tool::yaw_from_quat(ctx_.vehicle_att.q);

          if (!std::isfinite(yaw)) {
            return;
          }

          ctx_.yaw = yaw;

          if (ctx_.home_inited &&
              !ctx_.home_yaw_inited) {
            ctx_.home_yaw = yaw;
            ctx_.home_yaw_inited = true;
          }
        });

    vehicle_land_sub_ =
      create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected",
        px4_qos,
        [this](
          const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
        {
          ctx_.land_detected = *msg;
        });

    ground_cmd_sub_ =
      create_subscription<ground_station_msgs::msg::GroundCommand>(
        "/ground_station/cmd_parsed",
        10,
        std::bind(
          &SnakeTest::groundCommandCallback,
          this,
          std::placeholders::_1));

    down_servo_sub_ =
      create_subscription<custom_vision_msgs::msg::ServoTargetArray>(
        "/vision/down/servo_targets",
        10,
        std::bind(
          &SnakeTest::downServoCallback,
          this,
          std::placeholders::_1));
  }

  void downServoCallback(
    const custom_vision_msgs::msg::ServoTargetArray::SharedPtr msg)
  {
    if (!mission_started_) {
      return;
    }

    const uint64_t now_us =
      static_cast<uint64_t>(now().nanoseconds() / 1000ULL);

    ctx_.vision_last_update_us = now_us;
    ctx_.vision_down_targets_stamp_us = now_us;
    ctx_.vision_down_targets.clear();

    if (msg->targets.empty()) {
      ctx_.vision_offset = VisionOffset{};
      return;
    }

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

      if (!read_track_id(target.x, track_id) ||
          id_count[track_id] != 1) {
        continue;
      }

      VisionOffset vo;
      vo.track_id = track_id;
      vo.dx = target.dx;
      vo.dy = target.dy;
      vo.type = target.type;
      vo.score = target.score;
      vo.stamp_us = now_us;

      const float center_cost = target.dx * target.dx + target.dy * target.dy;
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

    ctx_.vision_offset = ctx_.vision_down_targets.front();

    /*
     * 巡检过程中一旦收到新的有效目标，立即请求进入下视补盲中断。
     * 同一个 track_id 只触发一次，避免补盲结束后被同一目标反复打断。
     */
    if (sched_.current_name() == "SNAKE_GRID" &&
        !ctx_.down_align_request &&
        !animal_report_pending_ &&
        reported_animal_tracks_.count(ctx_.vision_offset.track_id) == 0) {
      pending_interrupt_track_id_ = ctx_.vision_offset.track_id;
      pending_animal_target_ = ctx_.vision_offset;
      ctx_.down_align_request = true;

      RCLCPP_WARN(
        get_logger(),
        "[VISION_DOWN] target detected -> interrupt: id=%d type=%d "
        "dx=%.3f dy=%.3f score=%.1f",
        ctx_.vision_offset.track_id,
        ctx_.vision_offset.type,
        ctx_.vision_offset.dx,
        ctx_.vision_offset.dy,
        ctx_.vision_offset.score);
    }
  }

  bool currentPositionToCell(
    int& ix,
    int& iy,
    float& animal_x,
    float& animal_y,
    std::string& cell_name) const
  {
    if (!ctx_.pos_valid() || !ctx_.home_inited || snake_cell_size_ <= 0.0) {
      return false;
    }

    /*
     * 补盲纠偏完成后，无人机已经位于动物正上方。
     * 因此动物平面位置直接取纠偏结束时的无人机本地位置，
     * 不再使用 dx/dy 做二次位置补偿。
     */
    animal_x = ctx_.cx();
    animal_y = ctx_.cy();

    ix = static_cast<int>(std::lround(
      (static_cast<double>(animal_x) - ctx_.origin_dx) /
      snake_cell_size_));

    iy = static_cast<int>(std::lround(
      (static_cast<double>(animal_y) - ctx_.origin_dy) /
      snake_cell_size_));

    if (ix < 0 || ix >= snake_x_cells_ ||
        iy < 0 || iy >= snake_y_cells_) {
      return false;
    }

    const int a = snake_x_cells_ - ix;
    const int b = iy + 1;
    cell_name = "A" + std::to_string(a) + "B" + std::to_string(b);
    return true;
  }

void publishAnimalInfoAfterAlign()
{
  if (!animal_report_pending_) {
    return;
  }

  const auto target = pending_animal_target_;

  int ix = -1;
  int iy = -1;
  float animal_x = 0.0f;
  float animal_y = 0.0f;
  std::string cell_name;

  if (!currentPositionToCell(ix, iy, animal_x, animal_y, cell_name)) {
    RCLCPP_WARN(
      get_logger(),
      "[ANIMAL] failed to calculate cell: pos=(%.2f %.2f)",
      ctx_.cx(),
      ctx_.cy());

    animal_report_pending_ = false;
    pending_animal_target_ = VisionOffset{};
    return;
  }

  if (reported_animal_tracks_.count(target.track_id) > 0) {
    RCLCPP_WARN(
      get_logger(),
      "[ANIMAL] track already reported: id=%d",
      target.track_id);

    animal_report_pending_ = false;
    pending_animal_target_ = VisionOffset{};
    return;
  }

  ground_station_msgs::msg::AnimalInfo msg;

  msg.plan_id =
    snake_task_ != nullptr && snake_task_->planReady()
    ? snake_task_->planId()
    : 0;

  msg.cell = cell_name;
  msg.animal_type = static_cast<uint8_t>(target.type);
  msg.count = 1;

  animal_info_pub_->publish(msg);
  reported_animal_tracks_.insert(target.track_id);

  RCLCPP_WARN(
    get_logger(),
    "[ANIMAL] published: plan=%u cell=%s type=%u count=%u "
    "track=%d grid=(%d,%d) pos=(%.2f %.2f)",
    static_cast<unsigned>(msg.plan_id),
    msg.cell.c_str(),
    static_cast<unsigned>(msg.animal_type),
    static_cast<unsigned>(msg.count),
    target.track_id,
    ix,
    iy,
    animal_x,
    animal_y);

  animal_report_pending_ = false;
  pending_animal_target_ = VisionOffset{};
}

  void groundCommandCallback(
  const ground_station_msgs::msg::GroundCommand::SharedPtr msg)
{
  const uint8_t command = msg->cmd_type;

  if (command ==
      ground_station_msgs::msg::GroundCommand::CMD_START) {
    if (mission_started_) {
      RCLCPP_WARN(get_logger(), "[GCS] mission already started");
      return;
    }

    if (!parseNoFlyCells(msg->no_fly_cells)) {
      RCLCPP_ERROR(get_logger(), "[GCS] invalid no-fly cells");
      return;
    }

    buildScheduler();
    mission_started_ = true;

    RCLCPP_WARN(
      get_logger(),
      "[GCS] START: no_fly=%zu",
      obstacle_cells_.size());

    return;
  }

  if (command ==
      ground_station_msgs::msg::GroundCommand::CMD_PAUSE) {
    RCLCPP_WARN(get_logger(), "[GCS] PAUSE");
    return;
  }

  if (command ==
      ground_station_msgs::msg::GroundCommand::CMD_RESUME) {
    RCLCPP_WARN(get_logger(), "[GCS] RESUME");
    return;
  }

  if (command ==
      ground_station_msgs::msg::GroundCommand::CMD_RTL) {
    RCLCPP_WARN(get_logger(), "[GCS] RTL");
    return;
  }

  if (command ==
      ground_station_msgs::msg::GroundCommand::CMD_LAND) {
    ctx_.handover_to_px4_land = true;
    RCLCPP_WARN(get_logger(), "[GCS] LAND");
    return;
  }

  if (command ==
      ground_station_msgs::msg::GroundCommand::CMD_RESET) {
    mission_started_ = false;
    last_published_plan_id_ = 0;
    pending_interrupt_track_id_ = -1;
    last_interrupt_track_id_ = -1;
    ctx_.down_align_request = false;
    ctx_.vision_offset = VisionOffset{};
    ctx_.vision_down_targets.clear();
    pending_animal_target_ = VisionOffset{};
    animal_report_pending_ = false;
    reported_animal_tracks_.clear();
    obstacle_cells_.clear();
    buildScheduler();

    RCLCPP_WARN(get_logger(), "[GCS] RESET");
    return;
  }

  if (command ==
      ground_station_msgs::msg::GroundCommand::CMD_CLEAR_TRACK) {
    ctx_.detected_targets.clear();
    pending_interrupt_track_id_ = -1;
    last_interrupt_track_id_ = -1;
    ctx_.down_align_request = false;
    ctx_.vision_offset = VisionOffset{};
    ctx_.vision_down_targets.clear();
    pending_animal_target_ = VisionOffset{};
    animal_report_pending_ = false;
    reported_animal_tracks_.clear();
    RCLCPP_WARN(get_logger(), "[GCS] CLEAR_TRACK");
  }
}
bool parseNoFlyCells(
  const std::vector<std::string>& names)
{
  obstacle_cells_.clear();

  for (std::string name : names) {
    std::transform(
      name.begin(),
      name.end(),
      name.begin(),
      [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
      });

    const auto b_pos = name.find('B');

    if (name.empty() ||
        name.front() != 'A' ||
        b_pos == std::string::npos) {
      return false;
    }

    int a;
    int b;

    try {
      a = std::stoi(
        name.substr(1, b_pos - 1));

      b = std::stoi(
        name.substr(b_pos + 1));
    } catch (...) {
      return false;
    }

    if (a < 1 || a > snake_x_cells_ ||
        b < 1 || b > snake_y_cells_) {
      return false;
    }

    /*
     * A9B1 -> (0,0)
     * A8B1 -> (1,0)
     * A1B7 -> (8,6)
     */
    const int ix = snake_x_cells_ - a;
    const int iy = b - 1;

    if (ix == 0 && iy == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "[GCS] A9B1 is the takeoff cell");
      return false;
    }

    const bool duplicate = std::any_of(
      obstacle_cells_.begin(),
      obstacle_cells_.end(),
      [ix, iy](const SnakeGridTask::ObstacleCell& cell) {
        return cell.ix == ix && cell.iy == iy;
      });

    if (!duplicate) {
      obstacle_cells_.push_back(
        SnakeGridTask::ObstacleCell{ix, iy});
    }
  }

  return true;
}//增加字符串禁飞区解析
SnakeGridTask::FirstAxis chooseSnakeFirstAxis() const
{
  if (obstacle_cells_.size() < 2) {
    return SnakeGridTask::FirstAxis::X_FIRST;
  }

  const int first_ix = obstacle_cells_.front().ix;

  const bool along_y = std::all_of(
    obstacle_cells_.begin(),
    obstacle_cells_.end(),
    [first_ix](const SnakeGridTask::ObstacleCell& cell) {
      return cell.ix == first_ix;
    });

  return along_y
    ? SnakeGridTask::FirstAxis::Y_FIRST
    : SnakeGridTask::FirstAxis::X_FIRST;
}
  void buildScheduler()
  {
    sched_.clear();
    snake_task_ = nullptr;

    sched_.add(
      std::make_unique<WaitHomeTask>(
        get_logger()));

    sched_.add(
      std::make_unique<PresetpointTask>(
        get_logger()));

    sched_.add(
      std::make_unique<SetOffboardTask>(
        get_logger(),
        *px4_));

    sched_.add(
      std::make_unique<ArmTask>(
        get_logger(),
        *px4_));

    sched_.add(
      std::make_unique<TakeoffTask>(
        get_logger(),
        *px4_,
        arrival_error_max_));

    sched_.add(
      std::make_unique<HoverTask>(
        get_logger(),
        2.0));

    SnakeGridTask::Config snake_cfg;

    const auto first_axis =
      chooseSnakeFirstAxis();

    snake_cfg.first_axis = first_axis;

    RCLCPP_INFO(
      get_logger(),
      "[SNAKE] selected axis=%s",
      first_axis == SnakeGridTask::FirstAxis::Y_FIRST
        ? "Y_FIRST"
        : "X_FIRST");
    snake_cfg.stop_mode =
      SnakeGridTask::StopMode::LINE_END_ONLY;

    snake_cfg.x_cells = snake_x_cells_;
    snake_cfg.y_cells = snake_y_cells_;
    snake_cfg.cell_size = snake_cell_size_;

    snake_cfg.obstacle_cells = obstacle_cells_;

    snake_cfg.include_start_cell = true;
    snake_cfg.hover_s = snake_hover_s_;
    snake_cfg.max_step_m = snake_max_step_m_;
    snake_cfg.arrive_xy_m = snake_arrive_xy_m_;
    snake_cfg.arrive_z_m = snake_arrive_z_m_;

    snake_cfg.x_sign = 1;
    snake_cfg.y_sign = 1;

    auto snake_task =
      std::make_unique<SnakeGridTask>(
        get_logger(),
        snake_cfg);

    snake_task_ = snake_task.get();

    sched_.add(std::move(snake_task));

    /*
     * X_FIRST蛇形路线的正常结束网格。
     *
     * y_cells为奇数：
     *   最后一行正向扫描，结束在右端。
     *
     * y_cells为偶数：
     *   最后一行反向扫描，结束在左端。
     */
    int snake_end_x = 0;
    int snake_end_y = 0;

    if (first_axis ==
        SnakeGridTask::FirstAxis::X_FIRST) {
      /*
      * 一行一行沿x扫描。
      */
      snake_end_x =
        snake_y_cells_ % 2 == 1
        ? snake_x_cells_ - 1
        : 0;

      snake_end_y =
        snake_y_cells_ - 1;
    } else {
      /*
      * 一列一列沿y扫描。
      */
      snake_end_x =
        snake_x_cells_ - 1;

      snake_end_y =
        snake_x_cells_ % 2 == 1
        ? snake_y_cells_ - 1
        : 0;
    }

    offboard_core_pkg::AStarGotoTask::Config astar_cfg;

    astar_cfg.cols = snake_x_cells_;
    astar_cfg.rows = snake_y_cells_;
    astar_cfg.cell_size = snake_cell_size_;

    astar_cfg.start_cell = {
      snake_end_x,
      snake_end_y
    };

    astar_cfg.goal_cell = {0, 0};

    for (const auto& obstacle : obstacle_cells_) {
      astar_cfg.obstacle_cells.push_back(
        offboard_core_pkg::AStarPlanner::Cell{
          obstacle.ix,
          obstacle.iy});
    }

    astar_cfg.max_step_m = 0.03;
    astar_cfg.arrive_xy_m = 0.08;
    astar_cfg.arrive_z_m = 0.10;
    astar_cfg.hover_s = 0.40;

    astar_cfg.x_sign = 1;
    astar_cfg.y_sign = 1;

    sched_.add(
      std::make_unique<
        offboard_core_pkg::AStarGotoTask>(
        get_logger(),
        astar_cfg));

    offboard_core_pkg::Land45Task::Config land45_cfg;

    land45_cfg.approach_dir_x = 0.0;
    land45_cfg.approach_dir_y = 1.0;

    land45_cfg.approach_step_m = 0.04;
    land45_cfg.descend_step_m = 0.025;

    land45_cfg.approach_xy_tol_m = 0.08;
    land45_cfg.approach_z_tol_m = 0.08;

    land45_cfg.handover_height_m = 0.18;
    land45_cfg.max_horizontal_offset_m = 1.50;

    land45_cfg.approach_hold_s = 0.40;
    land45_cfg.led_toggle_s = 0.25;
    land45_cfg.timeout_s = 30.0;

    sched_.add(std::make_unique<offboard_core_pkg::Land45Task>(
      get_logger(),
      land45_cfg,
      landing_led_pub_));

    sched_.add(std::make_unique<offboard_core_pkg::Px4LandModeTask>(
      get_logger(),
      *px4_));

    sched_.addAux(
      std::make_unique<
        offboard_core_pkg::StartDownBlindCheckTask>(
        get_logger(),
        timeout_s_,
        align_tol_m_,
        15,
        k_img_to_meter_,
        max_step_m_,
        500.0,
        0.35,
        img_to_body_x_sign_,
        img_to_body_y_sign_));

    sched_.reset();
  }

  void handleDownAlignInterrupt()
  {
    if (!ctx_.down_align_request) {
      return;
    }

    if (sched_.current_name() != "SNAKE_GRID") {
      ctx_.down_align_request = false;
      pending_interrupt_track_id_ = -1;
      pending_animal_target_ = VisionOffset{};
      return;
    }

    if (sched_.interrupt("START_DOWN_BLIND_CHECK", ctx_)) {
      ctx_.down_align_request = false;
      last_interrupt_track_id_ = pending_interrupt_track_id_;
      pending_interrupt_track_id_ = -1;
      animal_report_pending_ = true;

      RCLCPP_WARN(
        get_logger(),
        "[SCHED] current task -> START_DOWN_BLIND_CHECK");

      return;
    }

    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[SCHED] failed to interrupt START_DOWN_BLIND_CHECK");
  }

  void publishVisionGates()
  {
    std_msgs::msg::Bool front_msg;
    front_msg.data = ctx_.vision_front_enable;
    vision_front_enable_pub_->publish(front_msg);

    std_msgs::msg::Bool down_msg;
    const std::string task = sched_.current_name();

    // 巡检阶段必须开启下视视觉，否则无法在发现目标时发起中断。
    down_msg.data =
      ctx_.vision_down_enable ||
      task == "SNAKE_GRID" ||
      task == "START_DOWN_BLIND_CHECK";

    vision_down_enable_pub_->publish(down_msg);
  }

  void publishMissionPlan()
  {
    if (snake_task_ == nullptr ||
        !snake_task_->planReady()) {
      return;
    }

    if (snake_task_->planId() ==
        last_published_plan_id_) {
      return;
    }

    ground_station_msgs::msg::MissionPlan msg;

    msg.plan_id = snake_task_->planId();
    msg.route_cells = snake_task_->routeCells();

    mission_plan_pub_->publish(msg);

    last_published_plan_id_ = msg.plan_id;

    RCLCPP_INFO(
      get_logger(),
      "[MISSION_PLAN] plan=%u route_size=%zu",
      static_cast<unsigned>(msg.plan_id),
      msg.route_cells.size());
  }

  uint8_t missionState() const
{
  const std::string task = sched_.current_name();

  if (sched_.done()) {
    return ground_station_msgs::msg::TaskStatus::STATE_FINISHED;
  }

  if (task == "TAKEOFF") {
    return ground_station_msgs::msg::TaskStatus::STATE_TAKEOFF;
  }

  if (task == "SNAKE_GRID") {
    return ground_station_msgs::msg::TaskStatus::STATE_PATROL;
  }

  if (task == "START_DOWN_BLIND_CHECK") {
    return ground_station_msgs::msg::TaskStatus::STATE_BLIND_CHECK;
  }

  if (task == "ASTAR_GOTO") {
    return ground_station_msgs::msg::TaskStatus::STATE_RETURNING;
  }

  if (task == "LAND_45" ||
      task == "PX4_LAND_MODE") {
    return ground_station_msgs::msg::TaskStatus::STATE_LANDING;
  }

  return ground_station_msgs::msg::TaskStatus::STATE_WAITING;
}

void publishTaskStatus()
{
  ground_station_msgs::msg::TaskStatus msg;

  msg.state = missionState();
  msg.mission_done = sched_.done();

  if (snake_task_ != nullptr &&
      snake_task_->planReady()) {
    msg.plan_id = snake_task_->planId();

    msg.current_index =
      static_cast<uint32_t>(
        snake_task_->currentIndex());

    msg.current_cell =
      snake_task_->currentCell();
  }

  task_status_pub_->publish(msg);
}

  void onTimer()
{
  const auto current_time = now();

  double dt =
    (current_time - last_time_).seconds();

  last_time_ = current_time;

  if (!std::isfinite(dt) || dt < 0.0) {
    dt = 0.0;
  }

  dt = std::min(dt, 0.20);

  /*
   * 等待地面站START命令。
   */
  if (!mission_started_) {
    publishVisionGates();
    publishTaskStatus();
    return;
  }

  if (!sched_.done() &&
      !ctx_.handover_to_px4_land) {
    px4_->publish_offboard_control_mode(ctx_);
  }

  handleDownAlignInterrupt();

  const std::string task_before_tick = sched_.current_name();
  sched_.tick(ctx_, dt);
  const std::string task_after_tick = sched_.current_name();

  if (task_before_tick == "START_DOWN_BLIND_CHECK" &&
      task_after_tick != "START_DOWN_BLIND_CHECK") {
    publishAnimalInfoAfterAlign();
  }

  publishMissionPlan();
  publishVisionGates();

  if (!sched_.done() &&
      !ctx_.handover_to_px4_land) {
    px4_->publish_setpoint_from_ctx(ctx_);
  }

  publishTaskStatus();

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "[SNAKE_TEST] task=%s done=%d pos=(%.2f %.2f %.2f) "
    "sp=(%.2f %.2f %.2f) cell=(r=%d,c=%d)",
    sched_.current_name().c_str(),
    sched_.done() ? 1 : 0,
    ctx_.cx(),
    ctx_.cy(),
    ctx_.cz(),
    ctx_.sp_x,
    ctx_.sp_y,
    ctx_.sp_z,
    ctx_.current_r,
    ctx_.current_c);
}

private:
  Context ctx_;

  std::unique_ptr<Px4Iface> px4_;
  Scheduler sched_;

  /*
   * Scheduler拥有SnakeGridTask对象。
   * 这里只保存非拥有指针，用于读取规划路线和执行进度。
   */
  SnakeGridTask* snake_task_{nullptr};

  uint32_t last_published_plan_id_{0};
  bool mission_started_{false};
  int32_t pending_interrupt_track_id_{-1};
  int32_t last_interrupt_track_id_{-1};
  VisionOffset pending_animal_target_{};
  bool animal_report_pending_{false};
  std::unordered_set<int32_t> reported_animal_tracks_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  // ===== 基础飞行参数 =====
  double takeoff_height_m_{1.0};
  double arrival_error_max_{0.25};

  // ===== 蛇形参数 =====
  int snake_x_cells_{9};
  int snake_y_cells_{7};

  double snake_cell_size_{0.5};
  double snake_hover_s_{0.5};
  double snake_max_step_m_{0.03};
  double snake_arrive_xy_m_{0.12};
  double snake_arrive_z_m_{0.12};

  std::vector<SnakeGridTask::ObstacleCell>
    obstacle_cells_;

  // ===== 下视补盲参数 =====
  double timeout_s_{4.0};
  double align_tol_m_{0.02};
  double k_img_to_meter_{0.45};
  double max_step_m_{0.20};
  double img_to_body_x_sign_{-1.0};
  double img_to_body_y_sign_{1.0};

  // ===== Publishers =====
  rclcpp::Publisher<
    px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;

  rclcpp::Publisher<
    ground_station_msgs::msg::TaskStatus>::SharedPtr
    task_status_pub_;

  rclcpp::Publisher<
    ground_station_msgs::msg::MissionPlan>::SharedPtr
    mission_plan_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    vision_front_enable_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    vision_down_enable_pub_;
  rclcpp::Publisher<
    ground_station_msgs::msg::AnimalInfo>::SharedPtr
    animal_info_pub_;

  rclcpp::Publisher<
    ground_station_msgs::msg::MissionResult>::SharedPtr
    mission_result_pub_;

  // ===== Subscribers =====
  rclcpp::Subscription<
    px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;

  rclcpp::Subscription<
    px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    local_pos_sub_;

  rclcpp::Subscription<
    px4_msgs::msg::VehicleAttitude>::SharedPtr
    vehicle_att_sub_;

  rclcpp::Subscription<
    px4_msgs::msg::VehicleLandDetected>::SharedPtr
    vehicle_land_sub_;

  rclcpp::Subscription<
    ground_station_msgs::msg::GroundCommand>::SharedPtr
    ground_cmd_sub_;

  rclcpp::Subscription<
    custom_vision_msgs::msg::ServoTargetArray>::SharedPtr
    down_servo_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr 
    landing_led_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<SnakeTest>());
  rclcpp::shutdown();

  return 0;
}