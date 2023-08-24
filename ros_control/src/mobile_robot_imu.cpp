/**
*
* ROS Simulation Online Course
* @uthor Min Htet Han (ROM Robotics)
* 
**/

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>


double imu_z_angle=0.0;
sensor_msgs::Imu imu;  

//imu callback: 
void imuCallback(const sensor_msgs::Imu& imu_data) {
    imu = imu_data;
    imu_z_angle = imu.angular_velocity.z;

}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "imu_node"); 
    ros::NodeHandle nh; 
    geometry_msgs::PoseStamped pose_estimate;
    std_msgs::Float64 yaw_msg;
    geometry_msgs::Quaternion quat_est;
    double dx_odom_est=0.0;
    double dy_odom_est=0.0;
    double dl_odom_est=0.0;
    
    double yaw_est=0.0;
    double x_est=0.0;
    double y_est=0.0;
    double x_est_old = 0.0;
    double y_est_old = 0.0;    
    
    ros::Rate timer(1/0.01);  
    ros::Subscriber imu_subscriber = nh.subscribe("/imu_data", 1, imuCallback);    
    
    ros::Publisher yaw_publisher = nh.advertise<std_msgs::Float64>("/true_yaw",1);
    
    
    
    while(ros::ok()) {
        

       yaw_est += 0.01*imu_z_angle; 
            if (yaw_est<-M_PI) yaw_est+= 2.0*M_PI; 
            if (yaw_est>M_PI) yaw_est-= 2.0*M_PI;        
        
        ROS_INFO("angle is %f ", yaw_est);
        yaw_msg.data = yaw_est; //publish the yaw estimate, for use/display
        yaw_publisher.publish(yaw_msg);
        
        timer.sleep();
        ros::spinOnce();
    }
  return 0;
}