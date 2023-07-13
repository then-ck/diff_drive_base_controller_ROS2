#include "diffdrive_ros2/base_controller.hpp"
#include <cstdlib>

namespace base_controller
{
base_controller::base_controller() : Node("base_controller")
{
	RCLCPP_INFO(this->get_logger(), "Phase 1 - Initializing node...");
	
	this->declare_parameter<double>("wheel_base", 0.283);
	this->get_parameter("wheel_base", wheel_base);
	this->declare_parameter<double>("wheel_diam", 0.0816);
	this->get_parameter("wheel_diam", wheel_diam);
	this->declare_parameter<double>("wheel_diam2", wheel_diam);
	this->get_parameter("wheel_diam2", wheel_diam2);

    // Read, Set parameters for wheels: 
    this->declare_parameter<std::string>("left_wheel_joint", "2");
    this->get_parameter("left_wheel_joint", left_joint_name);
	this->declare_parameter<std::string>("right_wheel_joint", "1");
    this->get_parameter("right_wheel_joint", right_joint_name);

	//Initialise subscriber: 
	RCLCPP_INFO(this->get_logger(), "Phase 1 - Creating subscriber");
	twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&base_controller::twist_cb, this, std::placeholders::_1));
	// Initialise publisher:
	RCLCPP_INFO(this->get_logger(), "Phase 1 - Creating publisher");
	joint_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 1);

	RCLCPP_INFO(this->get_logger(), "*****  Phase 1 Complete  *****  |  Publish to /cmd_vel now...");
	
}


base_controller::~base_controller( )
{
}


// Subscriber callback function
void base_controller::twist_cb( const geometry_msgs::msg::Twist::SharedPtr msg )
{
	auto new_msg = trajectory_msgs::msg::JointTrajectory();

	new_msg.header.frame_id = frame_id;
	new_msg.joint_names.resize( 2 );
	new_msg.points.resize( 1 );
	new_msg.points[0].velocities.resize( 2 );
	new_msg.joint_names[0] = left_joint_name;
	new_msg.joint_names[1] = right_joint_name;

	new_msg.header.stamp = this->get_clock()->now();

	new_msg.points[0].velocities[0] = ( 0.5 * msg->linear.x - msg->angular.z * wheel_base ) / wheel_diam;
	new_msg.points[0].velocities[1] = ( 0.5 * msg->linear.x + msg->angular.z * wheel_base ) / wheel_diam2;

	joint_traj_pub->publish( new_msg );
}
}


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<base_controller::base_controller>());
    rclcpp::shutdown();

    return 0;
}


