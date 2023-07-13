#ifndef _base_controller_hpp
#define _base_controller_hpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace base_controller
{
class base_controller : public rclcpp::Node
{
public:
	base_controller();
	~base_controller();


private:
	// Init subscriber:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
	// Init publisher:
	rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub; 
	
	std::string frame_id;
	std::string left_joint_name;
	std::string right_joint_name;
	double wheel_base;
	double wheel_diam;
	double wheel_diam2;

	void twist_cb( const geometry_msgs::msg::Twist::SharedPtr msg );
};
}

#endif /* _base_controller_hpp */
