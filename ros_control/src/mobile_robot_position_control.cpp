/**
*
* ROS Simulation Online Course
* @uthor Min Htet Han (ROM Robotics)
* 
**/

#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>																					
#include <geometry_msgs/Twist.h>	
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>		

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>							

void desire_pose_CB(const geometry_msgs::Pose2D::ConstPtr& msg);
void current_pose_CB(const nav_msgs::Odometry::ConstPtr& msg);

void waypoint2d_CB(const geometry_msgs::PoseStamped::ConstPtr& waypoint_msg);
void ekf_odom_CB(const nav_msgs::Path::ConstPtr& odom_msg);

float linear_error_fun(geometry_msgs::Pose2D current_pose, geometry_msgs::Pose2D desire_pose);
float angular_error_fun(geometry_msgs::Pose2D current_pose, geometry_msgs::Pose2D desire_pose);


bool STOP = true;														
geometry_msgs::Pose2D CurPose;												
geometry_msgs::Pose2D DesPose;	

double desire_pose_x;
double desire_pose_y;
double desire_yaw;
double cur_pose_x;
double cur_pose_y;
double cur_yaw;										

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_control_node");		
	ros::NodeHandle n;													
	ros::Subscriber desire_pose_sub = n.subscribe("/desire_pose", 5, desire_pose_CB);
	ros::Subscriber current_pose_sub = n.subscribe("/odom", 5, current_pose_CB);

	// waypoint follower
	ros::Subscriber waypoint_sub = n.subscribe("rom_robot_controller/waypoint_2d", 1, waypoint2d_CB);
	ros::Subscriber ekf_odom_sub = n.subscribe("slam_path", 1, ekf_odom_CB);

	ros::Publisher twist_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	
	ros::Rate loop_rate(10);											
	float error_linear = 0;													
	float error_angular = 0;													
	geometry_msgs::Twist cmdvel;										
	
							
	while (ros::ok() && n.ok() )										
	{
		ros::spinOnce();
		if (STOP == false)									
		{

			
			error_linear = linear_error_fun(CurPose, DesPose);					
			error_angular = angular_error_fun(CurPose, DesPose);
			    if (error_angular<M_PI) error_angular+= 2.0*M_PI; 
    			if (error_angular>M_PI) error_angular-= 2.0*M_PI; 					
			printf("Error linear: %f, Error angular: %f\n", error_linear, error_angular);
			if (abs(error_angular) > 0.1){
				if(abs(DesPose.x-CurPose.x) < 0.1 && abs(DesPose.y-CurPose.y) < 0.1){
					cmdvel.angular.z = 0.0;
					cmdvel.linear.x = 0.0;

				}
				else if(error_angular > 0.0 and error_angular < 3.0){
					cmdvel.angular.z = 0.3;
					cmdvel.linear.x  = 0.0;
				}
				else if(error_angular < 0.0 and error_angular > -3.0){
					cmdvel.angular.z = -0.3;
					cmdvel.linear.x  = 0.0;
				}
				else{
				cmdvel.angular.z = 0.3;
				cmdvel.linear.x = 0.0;}
			}
			else{cmdvel.linear.x = 0.1 * error_linear;
				 cmdvel.angular.z = 0.0;}
			
			
			// if (error_linear > 0.0)											
			// {
			// 	cmdvel.linear.x = 0.2 * error_linear;							
			// }
			// else
			// {
			// 	cmdvel.linear.x = 0;										
			// }
			
			
			// cmdvel.angular.z = 0.5 * error_angular;	
						
			twist_publisher.publish(cmdvel);									
		}
		else
		{
			printf("Waiting...\n");										
		}
		loop_rate.sleep();												
	}
}

void waypoint2d_CB(const geometry_msgs::PoseStamped::ConstPtr& waypoint_msg)
{
	desire_pose_x = waypoint_msg->pose.position.x;

	desire_pose_y = waypoint_msg->pose.position.y;

	tf2::Quaternion quat;
	tf2::fromMsg(waypoint_msg->pose.orientation, quat);
	tf2::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);

	desire_yaw = yaw;

	return;


}

void ekf_odom_CB(const nav_msgs::Path::ConstPtr& odom_msg)
{
	cur_pose_x = odom_msg->pose.position.x;
	cur_pose_y = odom_msg->pose.position.y;

	tf2::Quaternion q;
	tf2::fromMsg(waypoint_msg->poses.pose.orientation, q);
	tf2::Matrix3x3 mat_(q);
	double roll, pitch, yaw;
	mat_.getRPY(roll, pitch, yaw);

	cur_yaw = yaw;

	return;


}




void desire_pose_CB(const geometry_msgs::Pose2D::ConstPtr& msg)			
{
	STOP = false;														
	DesPose.x = msg->x;													
	DesPose.y = msg->y;
	ROS_INFO("x is %f and y is %f", msg->x, msg->y);
	return;
}


void current_pose_CB(const nav_msgs::Odometry::ConstPtr& msg)			
{
	CurPose.x = msg->pose.pose.position.x;
	CurPose.y = msg->pose.pose.position.y;
	tf::Quaternion q(
		msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    if (yaw<M_PI) yaw+= 2.0*M_PI; 
    if (yaw>M_PI) yaw-= 2.0*M_PI; 
   
    
    CurPose.theta = yaw;
     
												
	return;
}


float angular_error_fun(geometry_msgs::Pose2D current_pose, geometry_msgs::Pose2D desire_pose)
{
	float error_x = desire_pose.x - current_pose.x;									
	float error_y = desire_pose.y - current_pose.y;									
	float desire_theta = atan2(error_y, error_x); 										
	float error_theta = desire_theta - current_pose.theta;

	return error_theta;
}


float linear_error_fun(geometry_msgs::Pose2D current_pose, geometry_msgs::Pose2D desire_pose)
{
	
	float error_x = desire_pose.x - current_pose.x;									
	float error_y = desire_pose.y - current_pose.y;									 
	float error_theta = angular_error_fun(current_pose, desire_pose);							
	float error_linear =  pow( pow(error_x,2.0) + pow(error_y,2.0), 0.5 )*cos(error_theta); 

	
	return error_linear;
}