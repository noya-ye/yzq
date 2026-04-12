#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "trajectory_generator_pkg/generator.hpp"

using namespace std::chrono_literals;

class TrajectoryNode : public rclcpp::Node {
public:
    TrajectoryNode() : Node("trajectory_node")
    {
        pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        timer_ = create_wall_timer(
            50ms, std::bind(&TrajectoryNode::onTimer, this));

        generateTrajectory();

        RCLCPP_INFO(get_logger(), "Trajectory node started");
    }

private:
    void generateTrajectory()
    {
        std::vector<Eigen::Vector3d> waypoints = {
            {0, 0, -1},
            {1, 0, -1},
            {1, 1, -1},
            {0, 1, -1}
        };

        generator_.generate(waypoints);
        total_time_ = generator_.getTotalTime();

        start_time_ = now();
    }

    void onTimer()
    {
        double t = (now() - start_time_).seconds();

        if (t > total_time_) {
            t = total_time_;
        }

        auto pos = generator_.samplePosition(t);
        auto vel = generator_.sampleVelocity(t);

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = now().nanoseconds() / 1000;

        msg.position = {
        (float)pos.x(),
        (float)pos.y(),
        (float)pos.z()
        };
        msg.velocity = {
            (float)vel.x(),
            (float) vel.y(),
            (float) vel.z()
        };

        pub_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    TrajectoryGenerator generator_;

    rclcpp::Time start_time_;
    double total_time_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}