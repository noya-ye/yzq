#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <cstdint>
#include <cmath>

using namespace std::chrono_literals;

class Offboard1mNode : public rclcpp::Node
{
public:
    Offboard1mNode() : Node("offboard_1m_node"), offboard_setpoint_counter_(0)
    {
        offboard_control_mode_pub_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 10);

        trajectory_setpoint_pub_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command", 10);

        vehicle_local_position_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position",
                10,
                std::bind(&Offboard1mNode::vehicle_local_position_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            100ms, std::bind(&Offboard1mNode::timer_callback, this));

        hold_start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Offboard closed-loop descend node started.");
    }

private:
    enum class FlightPhase
    {
        TAKEOFF_HOLD,
        DESCEND,
        LAND_CMD,
        DONE
    };

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

    uint64_t offboard_setpoint_counter_;
    FlightPhase phase_{FlightPhase::TAKEOFF_HOLD};

    rclcpp::Time hold_start_time_;

    bool local_pos_received_{false};
    float current_x_{0.0f};
    float current_y_{0.0f};
    float current_z_{0.0f};   // NED: 向下为正，所以在空中一般是负数

    const float takeoff_height_z_ = -1.0f;   // 上升1m
    const double hold_duration_s_ = 10.0;

    bool land_cmd_sent_{false};

    uint64_t now_us()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
        local_pos_received_ = true;
    }

    float current_height_m() const
    {
        // NED: z=-1 -> 高度1m
        return -current_z_;
    }

    void publish_offboard_control_mode_position()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = now_us();
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_offboard_control_mode_velocity()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = now_us();
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_position_setpoint(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = now_us();
        msg.position = {x, y, z};
        msg.yaw = yaw;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_velocity_setpoint(float vx, float vy, float vz, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = now_us();
        msg.velocity = {vx, vy, vz};
        msg.yaw = yaw;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = now_us();

        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_command_pub_->publish(msg);
    }

    void arm()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0f);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    void set_offboard_mode()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f, 6.0f);
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
    }

    void land()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(this->get_logger(), "Land command sent");
    }

    float compute_descend_vz(float height_m) const
    {
        // NED里 vz > 0 表示向下
        if (height_m > 0.8f) {
            return 0.40f;   // 高处下降稍快
        } else if (height_m > 0.3f) {
            return 0.18f;   // 中低空下降变慢
        } else {
            return 0.08f;   // 接近地面极慢下降
        }
    }

    void timer_callback()
    {
        if (!local_pos_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /fmu/out/vehicle_local_position ...");
            return;
        }

        // 预热：先发一段设定点再进offboard
        if (offboard_setpoint_counter_ < 10) {
            publish_offboard_control_mode_position();
            publish_position_setpoint(0.0f, 0.0f, takeoff_height_z_, 0.0f);
            offboard_setpoint_counter_++;
            return;
        }

        if (offboard_setpoint_counter_ == 10) {
            publish_offboard_control_mode_position();
            publish_position_setpoint(0.0f, 0.0f, takeoff_height_z_, 0.0f);

            set_offboard_mode();
            arm();

            hold_start_time_ = this->now();
            offboard_setpoint_counter_++;
            return;
        }

        const float height = current_height_m();

        switch (phase_) {
        case FlightPhase::TAKEOFF_HOLD:
        {
            publish_offboard_control_mode_position();
            publish_position_setpoint(0.0f, 0.0f, takeoff_height_z_, 0.0f);

            const double hold_elapsed = (this->now() - hold_start_time_).seconds();

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "HOLD phase: height=%.2f m, elapsed=%.2f s",
                height, hold_elapsed);

            if (hold_elapsed >= hold_duration_s_) {
                phase_ = FlightPhase::DESCEND;
                RCLCPP_INFO(this->get_logger(), "Switch to DESCEND phase.");
            }
            break;
        }

        case FlightPhase::DESCEND:
        {
            // 方案A：低于0.3m时继续极慢下降
            // 方案B：低于0.3m时直接land
            if (height <= 0.3f) {
                phase_ = FlightPhase::LAND_CMD;
                RCLCPP_INFO(this->get_logger(), "Height <= 0.3m, switch to LAND_CMD phase.");
                break;
            }

            const float vz = compute_descend_vz(height);

            publish_offboard_control_mode_velocity();
            publish_velocity_setpoint(0.0f, 0.0f, vz, 0.0f);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "DESCEND phase: height=%.2f m, vz=%.2f m/s",
                height, vz);
            break;
        }

        case FlightPhase::LAND_CMD:
        {
            if (!land_cmd_sent_) {
                land();
                land_cmd_sent_ = true;
            }

            // 发送 land 后通常不再继续维持 offboard 速度控制
            // 这里给一点缓冲，不反复发 land
            phase_ = FlightPhase::DONE;
            break;
        }

        case FlightPhase::DONE:
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "DONE phase: waiting for PX4 landing process, current height=%.2f m",
                height);
            break;
        }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Offboard1mNode>());
    rclcpp::shutdown();
    return 0;
}