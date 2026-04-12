#include <array>
#include <chrono>
#include <cmath>
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
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
    px4_topic_ = this->declare_parameter<std::string>("px4_topic", "/fmu/in/vehicle_visual_odometry");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    use_odom_twist_ = this->declare_parameter<bool>("use_odom_twist", true);

    zero_position_on_start_ = this->declare_parameter<bool>("zero_position_on_start", false);

    pos_var_x_ = this->declare_parameter<double>("position_variance_x", 0.05);
    pos_var_y_ = this->declare_parameter<double>("position_variance_y", 0.05);
    pos_var_z_ = this->declare_parameter<double>("position_variance_z", 0.08);

    ori_var_roll_  = this->declare_parameter<double>("orientation_variance_roll", 0.02);
    ori_var_pitch_ = this->declare_parameter<double>("orientation_variance_pitch", 0.02);
    ori_var_yaw_   = this->declare_parameter<double>("orientation_variance_yaw", 0.05);

    vel_var_x_ = this->declare_parameter<double>("velocity_variance_x", 0.05);
    vel_var_y_ = this->declare_parameter<double>("velocity_variance_y", 0.05);
    vel_var_z_ = this->declare_parameter<double>("velocity_variance_z", 0.08);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&FastlioToPx4Node::odomCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "fastlio_to_px4 node started.");
    RCLCPP_INFO(this->get_logger(), "Subscribed odom topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing PX4 topic: %s", px4_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    using namespace fastlio_to_px4;

    // ---------------------------
    // 1) 读取 ROS ENU / FLU 数据
    // ---------------------------
    Vec3 pos_enu{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
    };

    Quat q_enu_flu{
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z
    };

    Vec3 vel_enu{0.0, 0.0, 0.0};
    if (use_odom_twist_) {
      vel_enu = Vec3{
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
      };
    }

    Vec3 ang_vel_flu{
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z
    };

    // ---------------------------
    // 2) 原点归零（可选）
    // ---------------------------
    if (zero_position_on_start_ && !origin_inited_) {
      origin_enu_ = pos_enu;
      origin_inited_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Zero origin initialized at ENU: [%.3f, %.3f, %.3f]",
                  origin_enu_.x, origin_enu_.y, origin_enu_.z);
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

    // ---------------------------
    // 4) 封装 PX4 VehicleOdometry
    // ---------------------------
    px4_msgs::msg::VehicleOdometry out{};

    const uint64_t now_us = this->get_clock()->now().nanoseconds() / 1000ULL;

    out.timestamp = now_us;

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      out.timestamp_sample = now_us;
    } else {
      const uint64_t sample_us =
        static_cast<uint64_t>(msg->header.stamp.sec) * 1000000ULL +
        static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000ULL;
      out.timestamp_sample = sample_us;
    }

    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

    out.position[0] = static_cast<float>(pos_ned.x);
    out.position[1] = static_cast<float>(pos_ned.y);
    out.position[2] = static_cast<float>(pos_ned.z);

    out.q[0] = static_cast<float>(q_ned_frd.w);
    out.q[1] = static_cast<float>(q_ned_frd.x);
    out.q[2] = static_cast<float>(q_ned_frd.y);
    out.q[3] = static_cast<float>(q_ned_frd.z);

    out.velocity[0] = static_cast<float>(vel_ned.x);
    out.velocity[1] = static_cast<float>(vel_ned.y);
    out.velocity[2] = static_cast<float>(vel_ned.z);

    out.angular_velocity[0] = static_cast<float>(ang_vel_frd.x);
    out.angular_velocity[1] = static_cast<float>(ang_vel_frd.y);
    out.angular_velocity[2] = static_cast<float>(ang_vel_frd.z);

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
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;

  std::string odom_topic_;
  std::string px4_topic_;
  double publish_rate_hz_{50.0};
  bool use_odom_twist_{true};
  bool zero_position_on_start_{false};

  double pos_var_x_{0.05};
  double pos_var_y_{0.05};
  double pos_var_z_{0.08};

  double ori_var_roll_{0.02};
  double ori_var_pitch_{0.02};
  double ori_var_yaw_{0.05};

  double vel_var_x_{0.05};
  double vel_var_y_{0.05};
  double vel_var_z_{0.08};

  bool origin_inited_{false};
  fastlio_to_px4::Vec3 origin_enu_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioToPx4Node>());
  rclcpp::shutdown();
  return 0;
}