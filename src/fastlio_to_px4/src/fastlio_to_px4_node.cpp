#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "fastlio_to_px4/frame_conversion.hpp"

using namespace std::chrono_literals;

class FastlioToPx4Node : public rclcpp::Node
{
public:
  FastlioToPx4Node() : Node("fastlio_to_px4_node")
  {
    odom_topic_ = this->declare_parameter<std::string>(
      "odom_topic", "/fastlio2/lio_odom");

    px4_topic_ = this->declare_parameter<std::string>(
      "px4_topic", "/fmu/in/vehicle_visual_odometry");

    // 关键：PX4 EKF 外部视觉输入不要跟随 FAST-LIO 抖动，默认稳定 30Hz 发布
    publish_rate_hz_ = this->declare_parameter<double>(
      "publish_rate_hz", 30.0);

    // 当前阶段建议 false，先不要融合 FAST-LIO 速度
    use_odom_twist_ = this->declare_parameter<bool>(
      "use_odom_twist", false);

    zero_position_on_start_ = this->declare_parameter<bool>(
      "zero_position_on_start", false);

    // 超过这个时间没有新 FAST-LIO odom，就停止向 PX4 发布，避免旧数据继续喂 EKF
    stale_timeout_s_ = this->declare_parameter<double>(
      "stale_timeout_s", 0.35);

    // 单帧位置跳变保护，防止 FAST-LIO 偶发跳点直接喂给 PX4
    max_position_jump_m_ = this->declare_parameter<double>(
      "max_position_jump_m", 1.00);

    // 先保守一点，不要让 PX4 过度相信外部定位
    pos_var_x_ = this->declare_parameter<double>("position_variance_x", 0.10);
    pos_var_y_ = this->declare_parameter<double>("position_variance_y", 0.10);
    pos_var_z_ = this->declare_parameter<double>("position_variance_z", 0.15);

    ori_var_roll_  = this->declare_parameter<double>("orientation_variance_roll", 0.05);
    ori_var_pitch_ = this->declare_parameter<double>("orientation_variance_pitch", 0.05);
    ori_var_yaw_   = this->declare_parameter<double>("orientation_variance_yaw", 0.10);

    vel_var_x_ = this->declare_parameter<double>("velocity_variance_x", 0.30);
    vel_var_y_ = this->declare_parameter<double>("velocity_variance_y", 0.30);
    vel_var_z_ = this->declare_parameter<double>("velocity_variance_z", 0.40);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&FastlioToPx4Node::odomCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic_, 10);

    const double safe_rate = std::max(1.0, publish_rate_hz_);
    const auto period = std::chrono::duration<double>(1.0 / safe_rate);

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FastlioToPx4Node::timerPublish, this));

    RCLCPP_INFO(this->get_logger(), "fastlio_to_px4 node started.");
    RCLCPP_INFO(this->get_logger(), "Subscribed odom topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing PX4 topic: %s", px4_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_rate_hz: %.1f", publish_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "use_odom_twist: %s", use_odom_twist_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "stale_timeout_s: %.3f", stale_timeout_s_);
    RCLCPP_INFO(this->get_logger(), "max_position_jump_m: %.3f", max_position_jump_m_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->pose.pose.position.z)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[fastlio_to_px4] reject odom: position contains NaN/Inf");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    latest_odom_rx_time_ = this->now();
    has_latest_odom_ = true;
  }

  void timerPublish()
  {
    nav_msgs::msg::Odometry odom;
    rclcpp::Time rx_time;
    bool has_odom = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_latest_odom_) {
        odom = latest_odom_;
        rx_time = latest_odom_rx_time_;
        has_odom = true;
      }
    }

    if (!has_odom) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[fastlio_to_px4] waiting for odom: %s",
        odom_topic_.c_str());
      return;
    }

    const double age_s = (this->now() - rx_time).seconds();

    if (age_s > stale_timeout_s_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[fastlio_to_px4] latest odom too old: %.3f s > %.3f s, skip publish",
        age_s,
        stale_timeout_s_);
      return;
    }

    publishVehicleOdometry(odom);
  }

  void publishVehicleOdometry(const nav_msgs::msg::Odometry& msg)
  {
    using namespace fastlio_to_px4;

    // ---------------------------
    // 1) 读取 ROS ENU / FLU 数据
    // ---------------------------
    Vec3 pos_enu{
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z
    };

    Quat q_enu_flu{
      msg.pose.pose.orientation.w,
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z
    };

    Vec3 vel_enu{
      0.0,
      0.0,
      0.0
    };

    Vec3 ang_vel_flu{
      0.0,
      0.0,
      0.0
    };

    if (use_odom_twist_) {
      vel_enu = Vec3{
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z
      };

      ang_vel_flu = Vec3{
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z
      };
    }

    // ---------------------------
    // 2) 原点归零，可选
    // ---------------------------
    if (zero_position_on_start_ && !origin_inited_) {
      origin_enu_ = pos_enu;
      origin_inited_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Zero origin initialized at ENU: [%.3f, %.3f, %.3f]",
        origin_enu_.x,
        origin_enu_.y,
        origin_enu_.z);
    }

    if (zero_position_on_start_ && origin_inited_) {
      pos_enu.x -= origin_enu_.x;
      pos_enu.y -= origin_enu_.y;
      pos_enu.z -= origin_enu_.z;
    }

    // ---------------------------
    // 3) ENU/FLU -> NED/FRD
    // ---------------------------
    Vec3 pos_ned = enu_to_ned(pos_enu);
    Vec3 vel_ned = enu_to_ned(vel_enu);
    Vec3 ang_vel_frd = flu_to_frd(ang_vel_flu);

    Quat q_ned_frd = attitude_enu_flu_to_ned_frd(q_enu_flu);

    if (!std::isfinite(pos_ned.x) ||
        !std::isfinite(pos_ned.y) ||
        !std::isfinite(pos_ned.z)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[fastlio_to_px4] reject converted odom: NED position contains NaN/Inf");
      return;
    }

    // ---------------------------
    // 4) 大跳变保护
    // ---------------------------
    if (has_last_pub_pos_) {
      const double dx = pos_ned.x - last_pub_pos_ned_.x;
      const double dy = pos_ned.y - last_pub_pos_ned_.y;
      const double dz = pos_ned.z - last_pub_pos_ned_.z;
      const double jump = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (jump > max_position_jump_m_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "[fastlio_to_px4] reject odom jump: %.3f m > %.3f m, pos_ned=[%.3f %.3f %.3f], last=[%.3f %.3f %.3f]",
          jump,
          max_position_jump_m_,
          pos_ned.x,
          pos_ned.y,
          pos_ned.z,
          last_pub_pos_ned_.x,
          last_pub_pos_ned_.y,
          last_pub_pos_ned_.z);
        return;
      }
    }

    last_pub_pos_ned_ = pos_ned;
    has_last_pub_pos_ = true;

    // ---------------------------
    // 5) 封装 PX4 VehicleOdometry
    // ---------------------------
    px4_msgs::msg::VehicleOdometry out{};

    const uint64_t now_us = this->get_clock()->now().nanoseconds() / 1000ULL;

    // 为了减少 header.stamp 与 PX4 时间基不一致导致的 EV 延迟异常，
    // 这里先统一 timestamp 和 timestamp_sample。
    out.timestamp = now_us;
    out.timestamp_sample = now_us;

    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

    out.position[0] = static_cast<float>(pos_ned.x);
    out.position[1] = static_cast<float>(pos_ned.y);
    out.position[2] = static_cast<float>(pos_ned.z);

    out.q[0] = static_cast<float>(q_ned_frd.w);
    out.q[1] = static_cast<float>(q_ned_frd.x);
    out.q[2] = static_cast<float>(q_ned_frd.y);
    out.q[3] = static_cast<float>(q_ned_frd.z);

    if (use_odom_twist_) {
      out.velocity[0] = static_cast<float>(vel_ned.x);
      out.velocity[1] = static_cast<float>(vel_ned.y);
      out.velocity[2] = static_cast<float>(vel_ned.z);

      out.angular_velocity[0] = static_cast<float>(ang_vel_frd.x);
      out.angular_velocity[1] = static_cast<float>(ang_vel_frd.y);
      out.angular_velocity[2] = static_cast<float>(ang_vel_frd.z);
    } else {
      const float nan_f = std::numeric_limits<float>::quiet_NaN();

      out.velocity[0] = nan_f;
      out.velocity[1] = nan_f;
      out.velocity[2] = nan_f;

      out.angular_velocity[0] = nan_f;
      out.angular_velocity[1] = nan_f;
      out.angular_velocity[2] = nan_f;
    }

    out.position_variance[0] = static_cast<float>(pos_var_x_);
    out.position_variance[1] = static_cast<float>(pos_var_y_);
    out.position_variance[2] = static_cast<float>(pos_var_z_);

    out.orientation_variance[0] = static_cast<float>(ori_var_roll_);
    out.orientation_variance[1] = static_cast<float>(ori_var_pitch_);
    out.orientation_variance[2] = static_cast<float>(ori_var_yaw_);

    out.velocity_variance[0] = static_cast<float>(vel_var_x_);
    out.velocity_variance[1] = static_cast<float>(vel_var_y_);
    out.velocity_variance[2] = static_cast<float>(vel_var_z_);

    out.reset_counter = 0;

    pub_->publish(out);

    publish_count_++;
    if (publish_count_ % 90 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "[fastlio_to_px4] pub odom age=%.3f pos_ned=[%.3f %.3f %.3f] rate=%.1fHz twist=%d",
        (this->now() - latest_odom_rx_time_).seconds(),
        pos_ned.x,
        pos_ned.y,
        pos_ned.z,
        publish_rate_hz_,
        use_odom_twist_ ? 1 : 0);
    }
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Time latest_odom_rx_time_;
  bool has_latest_odom_{false};

  std::string odom_topic_;
  std::string px4_topic_;

  double publish_rate_hz_{30.0};
  bool use_odom_twist_{false};
  bool zero_position_on_start_{false};
  double stale_timeout_s_{0.20};
  double max_position_jump_m_{1.00};

  double pos_var_x_{0.10};
  double pos_var_y_{0.10};
  double pos_var_z_{0.15};

  double ori_var_roll_{0.05};
  double ori_var_pitch_{0.05};
  double ori_var_yaw_{0.10};

  double vel_var_x_{0.30};
  double vel_var_y_{0.30};
  double vel_var_z_{0.40};

  bool origin_inited_{false};
  fastlio_to_px4::Vec3 origin_enu_{};

  bool has_last_pub_pos_{false};
  fastlio_to_px4::Vec3 last_pub_pos_ned_{};

  uint64_t publish_count_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioToPx4Node>());
  rclcpp::shutdown();
  return 0;
}