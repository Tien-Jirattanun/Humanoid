// c++ include
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ros include
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

// lib include
#include "position_control.hpp"

using namespace std::chrono_literals;

double kp = 0.9;
double ki = 0.02;
double kd = 0.3;
double u_max = 3.14;
double u_min = -3.14;

PositionControl JL_hip_r(kp, ki, kd, u_max, u_min);
PositionControl JL_hip_p(kp, ki, kd, u_max, u_min);
PositionControl JL_knee(kp, ki, kd, u_max, u_min);
PositionControl JL_ankle_r(kp, ki, kd, u_max, u_min);
PositionControl JL_hip_y(kp, ki, kd, u_max, u_min);
PositionControl JL_ankle_p(kp, ki, kd, u_max, u_min);
PositionControl JR_hip_r(kp, ki, kd, u_max, u_min);
PositionControl JR_hip_y(kp, ki, kd, u_max, u_min);
PositionControl JR_hip_p(kp, ki, kd, u_max, u_min);
PositionControl JR_knee(kp, ki, kd, u_max, u_min);
PositionControl JR_ankle_r(kp, ki, kd, u_max, u_min);
PositionControl JR_ankle_p(kp, ki, kd, u_max, u_min);

class VelocityPublisher : public rclcpp::Node
{
public:
	VelocityPublisher()
		: Node("velocity_publisher")
	{

		// velocity commands publisher
		velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

		// robot joint position subscribers
		position_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/joint_states", 10, std::bind(&VelocityPublisher::position_callback, this, _1));

		// desired position subscriber
		ref_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
			"/ref_position", 10, std::bind(&VelocityPublisher::ref_position_callback, this, _1));

		timer_ = this->create_wall_timer(10ms, std::bind(&VelocityPublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
		auto velocity_message = std_msgs::msg::Float64MultiArray();

		hanuman_velocity_[0] = JL_hip_r.PIDControl(hanuman_ref_position_[0], hanuman_position_[0]);
		hanuman_velocity_[1] = JL_hip_p.PIDControl(hanuman_ref_position_[1], hanuman_position_[1]);
		hanuman_velocity_[2] = JL_knee.PIDControl(hanuman_ref_position_[2], hanuman_position_[2]);
		hanuman_velocity_[3] = -1 * JL_ankle_r.PIDControl(hanuman_ref_position_[3], hanuman_position_[3]);
		hanuman_velocity_[4] = -1 * JL_hip_y.PIDControl(hanuman_ref_position_[4], hanuman_position_[4]);
		hanuman_velocity_[5] = -1 * JL_ankle_p.PIDControl(hanuman_ref_position_[5], hanuman_position_[5]);
		hanuman_velocity_[6] = JR_hip_r.PIDControl(hanuman_ref_position_[6], hanuman_position_[6]);
		hanuman_velocity_[7] = -1 * JR_hip_y.PIDControl(hanuman_ref_position_[7], hanuman_position_[7]);
		hanuman_velocity_[8] = JR_hip_p.PIDControl(hanuman_ref_position_[8], hanuman_position_[8]);
		hanuman_velocity_[9] = JR_knee.PIDControl(hanuman_ref_position_[9], hanuman_position_[9]);
		hanuman_velocity_[10] = -1 * JR_ankle_r.PIDControl(hanuman_ref_position_[10], hanuman_position_[10]);
		hanuman_velocity_[11] = -1 * JR_ankle_p.PIDControl(hanuman_ref_position_[11], hanuman_position_[11]);

		velocity_message.data = hanuman_velocity_;
		velocity_publisher_->publish(velocity_message);
	}

	// robot position callback
	void position_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		hanuman_position_ = msg->position;
	}

	// reference position callback
	void ref_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
	{
		hanuman_ref_position_ = msg->data;
	}

	// ros variables
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_position_subscriber_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;

	// control variables
	std::vector<double> hanuman_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> hanuman_ref_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> hanuman_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityPublisher>());
	rclcpp::shutdown();
	return 0;
}