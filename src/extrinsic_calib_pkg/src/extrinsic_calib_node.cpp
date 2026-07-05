#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <Eigen/Dense>

#include <vector>
#include <deque>
#include <optional>
#include <cmath>
#include <iostream>
#include <limits>

#include "extrinsic_calib_pkg/svd_solver.hpp"

using std::placeholders::_1;

class ExtrinsicCalibNode : public rclcpp::Node
{
public:
    ExtrinsicCalibNode() : Node("extrinsic_calib_node")
    {
        sub_cam_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/calib/camera_point", 50,
            std::bind(&ExtrinsicCalibNode::camCallback, this, _1));

        sub_world_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/calib/world_point", 50,
            std::bind(&ExtrinsicCalibNode::worldCallback, this, _1));

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/fastlio2/lio_odom", 100,
            std::bind(&ExtrinsicCalibNode::odomCallback, this, _1));

        RCLCPP_INFO(get_logger(), "Extrinsic Calib Node Started");
    }

private:
    struct TimedPoint
    {
        double t;
        Eigen::Vector3d p;
    };

    struct TimedOdom
    {
        double t;
        Eigen::Matrix3d R_lio;   // body -> world
        Eigen::Vector3d T_lio;   // body origin in world
    };

private:
    // ========= 数据缓存 =========
    std::deque<TimedPoint> cam_buffer_;
    std::deque<TimedOdom> odom_buffer_;

    // ========= 标定样本 =========
    std::vector<Eigen::Vector3d> Pc_;   // camera frame
    std::vector<Eigen::Vector3d> Pb_;   // body frame

    // ========= 参数 =========
    const double max_cam_dt_ = 0.03;     // 30 ms
    const double max_odom_dt_ = 0.05;    // 50 ms
    const size_t min_samples_ = 15;
    const size_t max_buffer_size_ = 300;

    bool solved_once_ = false;

    // ========= 订阅器 =========
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_cam_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_world_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

private:
    // =========================
    // 工具函数
    // =========================
    static double stampToSec(const builtin_interfaces::msg::Time & stamp)
    {
        return static_cast<double>(stamp.sec) +
               static_cast<double>(stamp.nanosec) * 1e-9;
    }

    static Eigen::Matrix3d quatToRot(double x, double y, double z, double w)
    {
        Eigen::Quaterniond q(w, x, y, z);
        q.normalize();
        return q.toRotationMatrix();
    }

    void trimBuffer()
    {
        while (cam_buffer_.size() > max_buffer_size_) {
            cam_buffer_.pop_front();
        }
        while (odom_buffer_.size() > max_buffer_size_) {
            odom_buffer_.pop_front();
        }
    }

    std::optional<TimedPoint> findClosestCam(double target_t) const
    {
        double best_dt = std::numeric_limits<double>::max();
        std::optional<TimedPoint> best;

        for (const auto & item : cam_buffer_) {
            double dt = std::fabs(item.t - target_t);
            if (dt < best_dt) {
                best_dt = dt;
                best = item;
            }
        }

        if (!best || best_dt > max_cam_dt_) {
            return std::nullopt;
        }
        return best;
    }

    std::optional<TimedOdom> findClosestOdom(double target_t) const
    {
        double best_dt = std::numeric_limits<double>::max();
        std::optional<TimedOdom> best;

        for (const auto & item : odom_buffer_) {
            double dt = std::fabs(item.t - target_t);
            if (dt < best_dt) {
                best_dt = dt;
                best = item;
            }
        }

        if (!best || best_dt > max_odom_dt_) {
            return std::nullopt;
        }
        return best;
    }

    // =========================
    // 回调：相机点
    // =========================
    void camCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        TimedPoint tp;
        tp.t = stampToSec(msg->header.stamp);
        tp.p = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);

        cam_buffer_.push_back(tp);
        trimBuffer();
    }

    // =========================
    // 回调：FAST-LIO odom
    // =========================
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto & q = msg->pose.pose.orientation;
        const auto & p = msg->pose.pose.position;

        TimedOdom to;
        to.t = stampToSec(msg->header.stamp);
        to.R_lio = quatToRot(q.x, q.y, q.z, q.w);
        to.T_lio = Eigen::Vector3d(p.x, p.y, p.z);

        odom_buffer_.push_back(to);
        trimBuffer();
    }

    // =========================
    // 回调：世界点
    // 以 world_point 的时间为基准，去找最近 cam / odom
    // =========================
    void worldCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        double t_world = stampToSec(msg->header.stamp);
        Eigen::Vector3d Pw(msg->point.x, msg->point.y, msg->point.z);

        auto cam_opt = findClosestCam(t_world);
        if (!cam_opt) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "No matched camera_point for world_point");
            return;
        }

        auto odom_opt = findClosestOdom(t_world);
        if (!odom_opt) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "No matched odom for world_point");
            return;
        }

        const Eigen::Vector3d & Pc = cam_opt->p;
        const Eigen::Matrix3d & R_lio = odom_opt->R_lio;
        const Eigen::Vector3d & T_lio = odom_opt->T_lio;

        // world -> body
        // Pw = R_lio * Pb + T_lio
        // => Pb = R_lio^T * (Pw - T_lio)
        Eigen::Vector3d Pb = R_lio.transpose() * (Pw - T_lio);

        Pc_.push_back(Pc);
        Pb_.push_back(Pb);

        RCLCPP_INFO(
            get_logger(),
            "Collected: %zu | Pc = [%.3f %.3f %.3f] | Pb = [%.3f %.3f %.3f]",
            Pc_.size(),
            Pc.x(), Pc.y(), Pc.z(),
            Pb.x(), Pb.y(), Pb.z()
        );

        if (Pc_.size() >= min_samples_) {
            solve();
        }
    }

    // =========================
    // SVD 求外参
    // Pb = R_cb * Pc + t_cb
    // =========================
    void solve()
    {
        if (Pc_.size() != Pb_.size() || Pc_.empty()) {
            RCLCPP_ERROR(get_logger(), "Invalid sample set");
            return;
        }

        Eigen::Matrix3d R_cb;
        Eigen::Vector3d t_cb;

        extrinsic_calib::solveSVD(Pc_, Pb_, R_cb, t_cb);

        std::cout << "\n========== EXTRINSIC RESULT ==========\n";
        std::cout << "R_cb:\n" << R_cb << "\n\n";
        std::cout << "t_cb:\n" << t_cb.transpose() << "\n";

        double mean_err = 0.0;
        double max_err = 0.0;

        for (size_t i = 0; i < Pc_.size(); ++i) {
            Eigen::Vector3d pb_est = R_cb * Pc_[i] + t_cb;
            double e = (pb_est - Pb_[i]).norm();
            mean_err += e;
            if (e > max_err) {
                max_err = e;
            }
        }

        mean_err /= static_cast<double>(Pc_.size());

        std::cout << "Samples   : " << Pc_.size() << "\n";
        std::cout << "Mean error: " << mean_err << " m\n";
        std::cout << "Max  error: " << max_err << " m\n";
        std::cout << "======================================\n";

        if (!solved_once_) {
            solved_once_ = true;
            RCLCPP_INFO(get_logger(), "First valid solve completed");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtrinsicCalibNode>());
    rclcpp::shutdown();
    return 0;
}