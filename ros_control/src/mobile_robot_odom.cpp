/**
*
* ROS Simulation Online Course
* @uthor Min Htet Han (ROM Robotics)
* 
**/

#include <ros/ros.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <stdio.h>  
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

const double R_LEFT_WHEEL = 0.1016 / 2.0;
const double R_RIGHT_WHEEL = 0.1016 / 2.0; 

const double TRACK = 0.2032;

bool joints_states_good = false;

nav_msgs::Odometry odom;
sensor_msgs::JointState joint_state;
ros::Publisher odom_pub;
ros::Subscriber joint_state_subscriber;

double new_left_wheel_ang, old_left_wheel_ang;
double new_right_wheel_ang, old_right_wheel_ang;
double time_new, time_old, dt;
double prv_x, prv_y;
ros::Time current_time;
double imu_msg;

geometry_msgs::Quaternion convert_imu_data_to_Quaternion(double imu_data) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(imu_data / 2.0);
    quaternion.w = cos(imu_data / 2.0);
    return (quaternion);
}

void imu_callback(const std_msgs::Float64::ConstPtr& msg){
    imu_msg = msg->data;
    ROS_INFO("angle %f ", imu_msg);
}

void joint_state_CB(const sensor_msgs::JointState& joint_states) {

    double dtheta_right, dtheta_left, ds, dpsi;
    int no_of_joints = joint_states.name.size();
    ROS_INFO("joint %d ", no_of_joints);
    int joint;
    int no_of_joints_found = 0;
    bool found_name = false;
    

    old_left_wheel_ang = new_left_wheel_ang;
    old_right_wheel_ang = new_right_wheel_ang;   
    time_old = time_new;
    current_time = ros::Time::now();
    time_new = current_time.toSec();
    dt = time_new - time_old;

    for (joint = 0; joint < no_of_joints; joint++) {

        std::string joint_name(joint_states.name[joint]);
        if (joint_name.compare("front_left_joint") == 0) {
            new_left_wheel_ang = joint_states.position[joint];              
            no_of_joints_found++;
            
        }
        if (joint_name.compare("front_right_joint") == 0) {
            new_right_wheel_ang = joint_states.position[joint];
            no_of_joints_found++;
            
        }
    }    ROS_INFO("right_wheel_ang is %f ", old_right_wheel_ang);
        ROS_INFO("left_wheel_ang is %f ", old_left_wheel_ang);

       if (!joints_states_good) {
        
            joints_states_good = true; 
            old_left_wheel_ang = new_left_wheel_ang;
            old_right_wheel_ang = new_right_wheel_ang;          
        
    }
    
   if (joints_states_good) {
        dtheta_left = (new_left_wheel_ang - old_left_wheel_ang)/dt; 
        dtheta_right = (new_right_wheel_ang - old_right_wheel_ang)/dt;
        odom.pose.pose.position.x = prv_x + (R_LEFT_WHEEL/2.0) * (dtheta_right+dtheta_left) * cos(imu_msg) *dt ;
        odom.pose.pose.position.y = prv_y + (R_LEFT_WHEEL/2.0) * (dtheta_right+dtheta_left) * sin(imu_msg) *dt;

        ROS_INFO("angle %f ", imu_msg);
        imu_msg += imu_msg;
        
        odom.pose.pose.orientation = convert_imu_data_to_Quaternion(imu_msg);
        
        
         
        odom.header.stamp = current_time;
        odom_pub.publish(odom);
        prv_x =  odom.pose.pose.position.x;
        prv_y =  odom.pose.pose.position.y;

        
}}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mobile_robot_odom");
    ros::NodeHandle nh; 

    current_time = ros::Time::now();
    time_new = current_time.toSec();
    time_old = time_new;


    //initialize odom with pose and twist defined as zero at start-up location
    odom.child_frame_id = "base_link";
    odom.header.frame_id = "mobile_robot";
    odom.header.stamp = current_time;
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    
    ros::Rate timer(100.0); // a 100Hz timer

    odom_pub = nh.advertise<nav_msgs::Odometry>("mobile_odom", 1);
    joint_state_subscriber = nh.subscribe("joint_states", 1, joint_state_CB);
    ros::Subscriber imu_sub = nh.subscribe("/true_yaw", 1, imu_callback);
    while (ros::ok()) {
        

        ros::spin();
        
    }
}
