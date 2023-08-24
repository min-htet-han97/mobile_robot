/**
*
* ROS Simulation Online Course
* @uthor Min Htet Han (ROM Robotics)
* 
**/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>

float linear,angular;
float radius = 0.0508;
float width = 0.2032;


void twist_CBack(const geometry_msgs::Twist::ConstPtr &msg){
	linear = msg->linear.x;
	angular = msg->angular.z;
	//ROS_INFO("linear is %f and angular is %f ",linear,angular);


}

int main(int argc, char **argv){
	ros::init(argc, argv, "twist_to_motor_publisher");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("mobile_vel", 100,twist_CBack);
	ros::Publisher front_left_wheel_pub = nh.advertise<std_msgs::Float64>("/MobileRobot/front_left_wheel_controller/command", 100);
	ros::Publisher front_right_wheel_pub = nh.advertise<std_msgs::Float64>("/MobileRobot/front_right_wheel_controller/command", 100);
	ros::Publisher back_left_wheel_pub = nh.advertise<std_msgs::Float64>("/MobileRobot/back_left_wheel_controller/command", 100);
	ros::Publisher back_right_wheel_pub = nh.advertise<std_msgs::Float64>("/MobileRobot/back_right_wheel_controller/command", 100);
	ros::Rate timer(5);
	std_msgs::Float64 right_wheel_velocity;
	std_msgs::Float64 left_wheel_velocity;


	

	while(ros::ok()){
		right_wheel_velocity.data = (2*linear + angular*width)/ (2*radius);
		left_wheel_velocity.data = (2*linear - angular*width) / (2*radius);
		front_left_wheel_pub.publish(left_wheel_velocity);
		front_right_wheel_pub.publish(right_wheel_velocity);
		back_left_wheel_pub.publish(left_wheel_velocity);
		back_right_wheel_pub.publish(right_wheel_velocity);
		ROS_INFO("left vel is %f and right vel is %f", left_wheel_velocity.data, right_wheel_velocity.data);
		ros::spinOnce();
		timer.sleep();

	}
	return 0;



}