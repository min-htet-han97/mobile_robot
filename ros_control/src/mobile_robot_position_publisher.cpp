/**
*
* ROS Simulation Online Course
* @uthor Min Htet Han (ROM Robotics)
* 
**/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pos_publisher");

  ros::NodeHandle n;

  ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose2D>("/desire_pose", 1);

  ros::Rate loop_rate(10);

  geometry_msgs::Pose2D msg;
  msg.x = 5.0;
  msg.y = 5.0;
  while (ros::ok())
  {
  pose_pub.publish(msg);

  ros::spinOnce();

  loop_rate.sleep();


  }


  return 0;
}

