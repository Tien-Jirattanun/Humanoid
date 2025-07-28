// c++ include
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>


// ros include
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

// lib include
#include "pid_control.hpp"

using namespace std::chrono_literals;

double kp = 5.0;
double ki = 0.0;
double kd = 4.0;
double u_max = 500.0;
double u_min = -200.0;

PIDControl JL_hip_r(kp, ki, kd, u_max, u_min);
PIDControl JL_hip_p(kp, ki, kd, u_max, u_min);
PIDControl JL_knee(kp, ki, kd, u_max, u_min);
PIDControl JL_ankle_r(kp, ki, kd, u_max, u_min);
PIDControl JL_hip_y(kp, ki, kd, u_max, u_min);
PIDControl JL_ankle_p(kp, ki, kd, u_max, u_min);
PIDControl JR_hip_r(kp, ki, kd, u_max, u_min);
PIDControl JR_hip_y(kp, ki, kd, u_max, u_min);
PIDControl JR_hip_p(kp, ki, kd, u_max, u_min);
PIDControl JR_knee(kp, ki, kd, u_max, u_min);
PIDControl JR_ankle_r(kp, ki, kd, u_max, u_min);
PIDControl JR_ankle_p(kp, ki, kd, u_max, u_min);

class VelocityPublisher : public rclcpp::Node
{
public:
	VelocityPublisher()
		: Node("velocity_publisher")
	{

		// // velocity sim commands publisher
		// velocity_sim_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

		// velocity commands publisher
		velocity_real_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
			"/group_goal_velocity", 10);

		// robot joint position subscribers
		position_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/joint_states", 10, std::bind(&VelocityPublisher::position_callback, this, _1));

		// desired position subscriber
		ref_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
			"/ref_position", 10, std::bind(&VelocityPublisher::ref_position_callback, this, _1));
		
		ref_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
			"/ref_velocity", 10, std::bind(&VelocityPublisher::ref_velocity_callback, this, _1));

		// enable moving
		enable_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
			"/enable", 10, std::bind(&VelocityPublisher::enable_callback, this, _1));

		timer_ = this->create_wall_timer(10ms, std::bind(&VelocityPublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
		auto velocity_message = std_msgs::msg::Int16MultiArray();

		hanuman_velocity_[0] = JL_hip_y.PIDControlFunction(hanuman_ref_position_[0], hanuman_position_[0]);
		hanuman_velocity_[1] = JL_hip_r.PIDControlFunction(hanuman_ref_position_[1], hanuman_position_[1]);
		hanuman_velocity_[2] = JL_hip_p.PIDControlFunction(hanuman_ref_position_[2], hanuman_position_[2]);
		hanuman_velocity_[3] = JL_knee.PIDControlFunction(hanuman_ref_position_[3], hanuman_position_[3]);
		hanuman_velocity_[4] = JL_ankle_p.PIDControlFunction(hanuman_ref_position_[4], hanuman_position_[4]);
		hanuman_velocity_[5] = JL_ankle_r.PIDControlFunction(hanuman_ref_position_[5], hanuman_position_[5]);

		hanuman_velocity_[6] = JR_hip_y.PIDControlFunction(hanuman_ref_position_[6], hanuman_position_[6]);
		hanuman_velocity_[7] = JR_hip_r.PIDControlFunction(hanuman_ref_position_[7], hanuman_position_[7]);
		hanuman_velocity_[8] = JR_hip_p.PIDControlFunction(hanuman_ref_position_[8], hanuman_position_[8]);
		hanuman_velocity_[9] = JR_knee.PIDControlFunction(hanuman_ref_position_[9], hanuman_position_[9]);
		hanuman_velocity_[10] = JR_ankle_p.PIDControlFunction(hanuman_ref_position_[10], hanuman_position_[10]);
		hanuman_velocity_[11] = JR_ankle_r.PIDControlFunction(hanuman_ref_position_[11], hanuman_position_[11]);

		//enable moving
		if (enable_ == 0)
		{
			for (int i = 0; i < 12; i++)
			{
				hanuman_velocity_[i] = 0;
			}
		}

		//Feed forward
		for (int i = 0; i < 12; i++)
		{
			hanuman_velocity_[i] += hanuman_ref_velocity_[i] * 0.5; 
		}

		// logging
		for (int i = 0; i < 12; i++)
		{
			// std::cout << "hanuman[" << i << "]" << " : " << hanuman_velocity_[i] << "rad/s" << std::endl;
			hanuman_velocity_command_[i] = static_cast<int16_t>((hanuman_velocity_[i] * 60 / (2 * M_PI)) / 0.229);
		}
		// std::cout << "---------------------------------" << std::endl;


		velocity_message.data = hanuman_velocity_command_;
		velocity_real_publisher_->publish(velocity_message);
		// velocity_sim_publisher_->publish(velocity_message);
	}

	void enable_callback(const std_msgs::msg::Int16::SharedPtr msg)
	{
		enable_ = msg->data;
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

	// reference position callback
	void ref_velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
	{
		hanuman_ref_velocity_ = msg->data;
	}

	// ros variables
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_position_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_velocity_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr enable_subscriber_;
	// rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_sim_publisher_;
	rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr velocity_real_publisher_;
	

	// control variables
	std::vector<double> hanuman_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> hanuman_ref_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> hanuman_ref_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> hanuman_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<int16_t> hanuman_velocity_command_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
	int enable_ = 1;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityPublisher>());
	rclcpp::shutdown();
	return 0;
}