#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "ego_2d_planner_pkg/msg/bspline2_d.hpp"

using namespace std::chrono_literals;
using ego_2d_planner_pkg::msg::Bspline2D;

class TrajServer2DNode : public rclcpp::Node
{
public:
  struct CtrlPoint
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  TrajServer2DNode()
  : Node("traj_server_2d_node")
  {
    bspline_topic_ = declare_parameter<std::string>(
      "bspline_topic", "/ego_2d_planner/bspline_2d");

    cmd_topic_ = declare_parameter<std::string>(
      "cmd_topic", "/position_cmd");

    frame_id_ = declare_parameter<std::string>(
      "frame_id", "camera_init");

    cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 50.0);
    time_forward_ = declare_parameter<double>("time_forward", 0.70);
    yaw_dot_max_ = declare_parameter<double>("yaw_dot_max", M_PI);
    hold_after_finish_ = declare_parameter<bool>("hold_after_finish", true);
    fixed_z_ = declare_parameter<double>("fixed_z", 1.0);
    use_msg_z_ = declare_parameter<bool>("use_msg_z", true);

    cmd_.kx[0] = declare_parameter<double>("pos_gain_x", 0.0);
    cmd_.kx[1] = declare_parameter<double>("pos_gain_y", 0.0);
    cmd_.kx[2] = declare_parameter<double>("pos_gain_z", 0.0);
    cmd_.kv[0] = declare_parameter<double>("vel_gain_x", 0.0);
    cmd_.kv[1] = declare_parameter<double>("vel_gain_y", 0.0);
    cmd_.kv[2] = declare_parameter<double>("vel_gain_z", 0.0);

    pos_cmd_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(cmd_topic_, 50);

    bspline_sub_ = create_subscription<Bspline2D>(
      bspline_topic_,
      10,
      std::bind(&TrajServer2DNode::bsplineCallback, this, std::placeholders::_1));

    const int period_ms = std::max(1, static_cast<int>(1000.0 / std::max(1.0, cmd_rate_hz_)));
    cmd_timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&TrajServer2DNode::cmdTimerCallback, this));

    last_cmd_time_ = now();
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;

    RCLCPP_WARN(get_logger(), "[TrajServer2D-Bspline] ready");
    RCLCPP_WARN(get_logger(), "[TrajServer2D-Bspline] input : %s", bspline_topic_.c_str());
    RCLCPP_WARN(get_logger(), "[TrajServer2D-Bspline] output: %s", cmd_topic_.c_str());
  }

private:
  static double wrapPi(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  void bsplineCallback(const Bspline2D::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (msg->pos_pts.size() < 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[TrajServer2D-Bspline] reject bspline: ctrl pts too few=%zu",
        msg->pos_pts.size());
      return;
    }

    if (msg->order != 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[TrajServer2D-Bspline] only cubic order=3 is supported now, got order=%d",
        msg->order);
      return;
    }

    if (!std::isfinite(msg->knot_span) || msg->knot_span <= 0.01) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[TrajServer2D-Bspline] reject bspline: invalid knot_span=%.3f",
        msg->knot_span);
      return;
    }

    ctrl_pts_.clear();
    ctrl_pts_.reserve(msg->pos_pts.size());

    const double z_default = std::isfinite(msg->fixed_z) ? msg->fixed_z : fixed_z_;

    for (const auto& p : msg->pos_pts) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        continue;
      }

      CtrlPoint c;
      c.x = p.x;
      c.y = p.y;
      c.z = (use_msg_z_ && std::isfinite(p.z)) ? p.z : z_default;
      ctrl_pts_.push_back(c);
    }

    if (ctrl_pts_.size() < 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[TrajServer2D-Bspline] reject bspline after finite filter: ctrl=%zu",
        ctrl_pts_.size());
      return;
    }

    traj_id_ = msg->traj_id;
    order_ = msg->order;
    knot_span_ = msg->knot_span;
    traj_duration_ = msg->duration > 0.0 ? msg->duration : durationFromCtrl();
    start_time_ = rclcpp::Time(msg->start_time);

    // If simulated /use_sim_time is not enabled correctly, msg start time may be zero.
    // In that case use local node time to avoid invalid negative t_cur.
    if (start_time_.nanoseconds() == 0) {
      start_time_ = now();
    }

    receive_traj_ = true;

    RCLCPP_WARN(
      get_logger(),
      "[TrajServer2D-Bspline] recv traj_id=%d ctrl=%zu dt=%.3f duration=%.2f frame=%s",
      traj_id_, ctrl_pts_.size(), knot_span_, traj_duration_, msg->header.frame_id.c_str());
  }

  double durationFromCtrl() const
  {
    if (ctrl_pts_.size() < 4) {
      return 0.0;
    }
    return static_cast<double>(ctrl_pts_.size() - 3) * knot_span_;
  }

  CtrlPoint evaluate(double t) const
  {
    CtrlPoint out;

    if (ctrl_pts_.empty()) {
      return out;
    }

    if (ctrl_pts_.size() < 4) {
      return ctrl_pts_.front();
    }

    const double dur = durationFromCtrl();
    t = std::clamp(t, 0.0, dur);

    int seg = static_cast<int>(std::floor(t / knot_span_));
    const int max_seg = static_cast<int>(ctrl_pts_.size()) - 4;
    if (seg > max_seg) {
      seg = max_seg;
    }

    const double u = std::clamp((t - static_cast<double>(seg) * knot_span_) / knot_span_, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;

    const double b0 = (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0;
    const double b1 = (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0;
    const double b2 = (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0;
    const double b3 = u3 / 6.0;

    const auto& p0 = ctrl_pts_[seg];
    const auto& p1 = ctrl_pts_[seg + 1];
    const auto& p2 = ctrl_pts_[seg + 2];
    const auto& p3 = ctrl_pts_[seg + 3];

    out.x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
    out.y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
    out.z = b0 * p0.z + b1 * p1.z + b2 * p2.z + b3 * p3.z;
    return out;
  }

  CtrlPoint evaluateVelocity(double t) const
  {
    CtrlPoint out;
    if (ctrl_pts_.size() < 4) {
      return out;
    }

    const double dur = durationFromCtrl();
    t = std::clamp(t, 0.0, dur);

    int seg = static_cast<int>(std::floor(t / knot_span_));
    const int max_seg = static_cast<int>(ctrl_pts_.size()) - 4;
    if (seg > max_seg) {
      seg = max_seg;
    }

    const double u = std::clamp((t - static_cast<double>(seg) * knot_span_) / knot_span_, 0.0, 1.0);
    const double u2 = u * u;

    const double b0 = (-3.0 + 6.0 * u - 3.0 * u2) / 6.0;
    const double b1 = (-12.0 * u + 9.0 * u2) / 6.0;
    const double b2 = (3.0 + 6.0 * u - 9.0 * u2) / 6.0;
    const double b3 = (3.0 * u2) / 6.0;

    const auto& p0 = ctrl_pts_[seg];
    const auto& p1 = ctrl_pts_[seg + 1];
    const auto& p2 = ctrl_pts_[seg + 2];
    const auto& p3 = ctrl_pts_[seg + 3];

    const double inv_dt = 1.0 / knot_span_;
    out.x = (b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x) * inv_dt;
    out.y = (b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y) * inv_dt;
    out.z = (b0 * p0.z + b1 * p1.z + b2 * p2.z + b3 * p3.z) * inv_dt;
    return out;
  }

  CtrlPoint evaluateAcceleration(double t) const
  {
    CtrlPoint out;
    if (ctrl_pts_.size() < 4) {
      return out;
    }

    const double dur = durationFromCtrl();
    t = std::clamp(t, 0.0, dur);

    int seg = static_cast<int>(std::floor(t / knot_span_));
    const int max_seg = static_cast<int>(ctrl_pts_.size()) - 4;
    if (seg > max_seg) {
      seg = max_seg;
    }

    const double u = std::clamp((t - static_cast<double>(seg) * knot_span_) / knot_span_, 0.0, 1.0);

    const double b0 = 1.0 - u;
    const double b1 = -2.0 + 3.0 * u;
    const double b2 = 1.0 - 3.0 * u;
    const double b3 = u;

    const auto& p0 = ctrl_pts_[seg];
    const auto& p1 = ctrl_pts_[seg + 1];
    const auto& p2 = ctrl_pts_[seg + 2];
    const auto& p3 = ctrl_pts_[seg + 3];

    const double inv_dt2 = 1.0 / (knot_span_ * knot_span_);
    out.x = (b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x) * inv_dt2;
    out.y = (b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y) * inv_dt2;
    out.z = (b0 * p0.z + b1 * p1.z + b2 * p2.z + b3 * p3.z) * inv_dt2;
    return out;
  }

  std::pair<double, double> calculateYaw(
    double t_cur,
    const CtrlPoint& pos,
    const rclcpp::Time& time_now,
    const rclcpp::Time& time_last)
  {
    const double dt = std::max(1e-3, (time_now - time_last).seconds());
    const double tf = std::min(traj_duration_, t_cur + std::max(0.05, time_forward_));
    const CtrlPoint pf = evaluate(tf);

    const double dx = pf.x - pos.x;
    const double dy = pf.y - pos.y;

    double yaw_target = last_yaw_;
    if (std::hypot(dx, dy) > 0.05) {
      yaw_target = std::atan2(dy, dx);
    }

    const double dyaw = wrapPi(yaw_target - last_yaw_);
    const double max_change = yaw_dot_max_ * dt;

    double yaw = last_yaw_;
    double yawdot = 0.0;

    if (dyaw > max_change) {
      yaw = wrapPi(last_yaw_ + max_change);
      yawdot = yaw_dot_max_;
    } else if (dyaw < -max_change) {
      yaw = wrapPi(last_yaw_ - max_change);
      yawdot = -yaw_dot_max_;
    } else {
      yaw = yaw_target;
      yawdot = dyaw / dt;
    }

    // Same style as original EGO traj_server: small LPF.
    yaw = wrapPi(0.5 * last_yaw_ + 0.5 * yaw);
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;

    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    return {yaw, yawdot};
  }

  void cmdTimerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!receive_traj_ || ctrl_pts_.size() < 4) {
      return;
    }

    const rclcpp::Time time_now = now();
    double t_cur = (time_now - start_time_).seconds();

    CtrlPoint pos;
    CtrlPoint vel;
    CtrlPoint acc;

    if (t_cur >= 0.0 && t_cur < traj_duration_) {
      pos = evaluate(t_cur);
      vel = evaluateVelocity(t_cur);
      acc = evaluateAcceleration(t_cur);
    } else if (t_cur >= traj_duration_) {
      pos = evaluate(traj_duration_);
      if (!hold_after_finish_) {
        vel = evaluateVelocity(traj_duration_);
        acc = evaluateAcceleration(traj_duration_);
      }
    } else {
      // Future trajectory start time. Hold first point until it becomes active.
      pos = evaluate(0.0);
    }

    const auto yaw_yawdot = calculateYaw(
      std::clamp(t_cur, 0.0, traj_duration_), pos, time_now, last_cmd_time_);
    last_cmd_time_ = time_now;

    cmd_.header.stamp = time_now;
    cmd_.header.frame_id = frame_id_;
    cmd_.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd_.trajectory_id = traj_id_;

    cmd_.position.x = pos.x;
    cmd_.position.y = pos.y;
    cmd_.position.z = pos.z;

    cmd_.velocity.x = vel.x;
    cmd_.velocity.y = vel.y;
    cmd_.velocity.z = vel.z;

    cmd_.acceleration.x = acc.x;
    cmd_.acceleration.y = acc.y;
    cmd_.acceleration.z = acc.z;

    cmd_.yaw = yaw_yawdot.first;
    cmd_.yaw_dot = yaw_yawdot.second;

    pos_cmd_pub_->publish(cmd_);
  }

private:
  std::mutex mutex_;

  std::string bspline_topic_{"/ego_2d_planner/bspline_2d"};
  std::string cmd_topic_{"/position_cmd"};
  std::string frame_id_{"camera_init"};

  double cmd_rate_hz_{50.0};
  double time_forward_{0.70};
  double yaw_dot_max_{M_PI};
  bool hold_after_finish_{true};
  double fixed_z_{1.0};
  bool use_msg_z_{true};

  bool receive_traj_{false};
  int traj_id_{0};
  int order_{3};
  double knot_span_{0.25};
  double traj_duration_{0.0};
  rclcpp::Time start_time_;
  rclcpp::Time last_cmd_time_;

  double last_yaw_{0.0};
  double last_yaw_dot_{0.0};

  std::vector<CtrlPoint> ctrl_pts_;
  quadrotor_msgs::msg::PositionCommand cmd_;

  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::Subscription<Bspline2D>::SharedPtr bspline_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajServer2DNode>());
  rclcpp::shutdown();
  return 0;
}
