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
#include "cascade_control.hpp"

using namespace std::chrono_literals;

double outer_kp = 5;
double outer_ki = 0.05;
double outer_kd = 0.2;
double outer_u_max = 3.14;
double outer_u_min = -3.14;

double inner_kp = 0.5;
double inner_ki = 0.1;
double inner_kd = 0.0;
double inner_u_max = 3.14;
double inner_u_min = -3.14;

CascadeControl JL_hip_r(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JL_hip_p(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JL_knee(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JL_ankle_r(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JL_hip_y(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JL_ankle_p(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JR_hip_r(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JR_hip_y(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JR_hip_p(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JR_knee(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JR_ankle_r(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);
CascadeControl JR_ankle_p(outer_kp, outer_ki, outer_kd, outer_u_max, outer_u_min, inner_kp, inner_ki, inner_kd, inner_u_max, inner_u_min);

class VelocityPublisher : public rclcpp::Node
{
public:
	VelocityPublisher()
		: Node("velocity_publisher")
	{
		// velocity commands publisher
		velocity_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

		// robot joint position subscribers
		position_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/joint_states", 10, std::bind(&VelocityPublisher::position_callback, this, _1));

		// robot joint velocity subscribers
		velocity_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/joint_states", 10, std::bind(&VelocityPublisher::velocity_callback, this, _1));
		
		// desired position subscriber
		ref_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
			"/ref_position", 10, std::bind(&VelocityPublisher::ref_position_callback, this, _1));

        ref_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/ref_velocity", 10, std::bind(&VelocityPublisher::ref_velocity_callback, this, _1));

		timer_ = this->create_wall_timer(10ms, std::bind(&VelocityPublisher::timer_callback, this));
	}
private:
	void timer_callback()
	{
		auto velocity_message = std_msgs::msg::Float64MultiArray();

        hanuman_velocity_command_[0] = JL_hip_y.CascadeControlFunction(hanuman_ref_position_[0], hanuman_ref_velocity_[0], hanuman_position_[0], hanuman_velocity_[0]);
		hanuman_velocity_command_[1] = JL_hip_r.CascadeControlFunction(hanuman_ref_position_[1], hanuman_ref_velocity_[1], hanuman_position_[1], hanuman_velocity_[1]);
		hanuman_velocity_command_[2] = JL_hip_p.CascadeControlFunction(hanuman_ref_position_[2], hanuman_ref_velocity_[2], hanuman_position_[2], hanuman_velocity_[2]);
		hanuman_velocity_command_[3] = JL_knee.CascadeControlFunction(hanuman_ref_position_[3], hanuman_ref_velocity_[3], hanuman_position_[3], hanuman_velocity_[3]);
		hanuman_velocity_command_[4] = JL_ankle_p.CascadeControlFunction(hanuman_ref_position_[4], hanuman_ref_velocity_[4], hanuman_position_[4], hanuman_velocity_[4]);
		hanuman_velocity_command_[5] = JL_ankle_r.CascadeControlFunction(hanuman_ref_position_[5], hanuman_ref_velocity_[5], hanuman_position_[5], hanuman_velocity_[5]);

		hanuman_velocity_command_[6] = JR_hip_y.CascadeControlFunction(hanuman_ref_position_[6], hanuman_ref_velocity_[6], hanuman_position_[6], hanuman_velocity_[6]);
		hanuman_velocity_command_[7] = JR_hip_r.CascadeControlFunction(hanuman_ref_position_[7], hanuman_ref_velocity_[7], hanuman_position_[7], hanuman_velocity_[7]);
		hanuman_velocity_command_[8] = JR_hip_p.CascadeControlFunction(hanuman_ref_position_[8], hanuman_ref_velocity_[8], hanuman_position_[8], hanuman_velocity_[8]);
		hanuman_velocity_command_[9] = JR_knee.CascadeControlFunction(hanuman_ref_position_[9], hanuman_ref_velocity_[9], hanuman_position_[9], hanuman_velocity_[9]);
		hanuman_velocity_command_[10] = JR_ankle_p.CascadeControlFunction(hanuman_ref_position_[10], hanuman_ref_velocity_[10], hanuman_position_[10], hanuman_velocity_[10]);
		hanuman_velocity_command_[11] = JR_ankle_r.CascadeControlFunction(hanuman_ref_position_[11], hanuman_ref_velocity_[11], hanuman_position_[11], hanuman_velocity_[11]);
       
		velocity_message.data = hanuman_velocity_command_;
		velocity_command_publisher_->publish(velocity_message);
	}

	// robot position callback
	void position_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		hanuman_position_ = msg->position;
	}

	void velocity_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		hanuman_velocity_ = msg->velocity;
	}

	// reference position callback
	void ref_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
	{
		hanuman_ref_position_ = msg->data;
	}

    // reference velocity callback
    void ref_velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        hanuman_ref_velocity_ = msg->data;
    }

	// ros variables
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr velocity_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_position_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_velocity_subscriber_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_command_publisher_;

	// control variables
	std::vector<double> hanuman_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> hanuman_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> hanuman_ref_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> hanuman_ref_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> hanuman_velocity_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityPublisher>());
	rclcpp::shutdown();
	return 0;
}
