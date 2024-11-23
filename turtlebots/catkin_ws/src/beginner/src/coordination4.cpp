#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Empty.h>
#include "nav_msgs/Odometry.h"
#include <fstream>
#include <cmath> 
// #include "turtlebot_controller.h"


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const char* path) {
    // Extract data from the Odometry message
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double yaw = atan2(2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);

    // Open a file for logging (append mode)
    std::ofstream logFile;
    logFile.open(path, std::ios::app);

    // Log the data to the file
    logFile << "x: " << x << ", y: " << y << ", t: " << yaw << std::endl;

    // Close the file
    logFile.close();
}

void odomCallback_3(const nav_msgs::Odometry::ConstPtr& msg) {
    odomCallback(msg, "/root/logs/logfile_3.txt");
}
void odomCallback_6(const nav_msgs::Odometry::ConstPtr& msg) {
    odomCallback(msg, "/root/logs/logfile_6.txt");
}
void odomCallback_7(const nav_msgs::Odometry::ConstPtr& msg) {
    odomCallback(msg, "/root/logs/logfile_7.txt");
}
void odomCallback_9(const nav_msgs::Odometry::ConstPtr& msg) {
    odomCallback(msg, "/root/logs/logfile_9.txt");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoid_obstacles");
    // Turtlebot_Controller controller;
    // ros::spin();

    ros::Publisher vel_pub_3, vel_pub_6, vel_pub_7, vel_pub_9;
    geometry_msgs::Twist linear_angular_vel_;
    ros::NodeHandle nh_;

    ros::Publisher reset_pub3 = nh_.advertise<std_msgs::Empty>("/bot3/reset", 1);
    ros::Publisher reset_pub6 = nh_.advertise<std_msgs::Empty>("/bot6/reset", 1);
    ros::Publisher reset_pub7 = nh_.advertise<std_msgs::Empty>("/bot7/reset", 1);
    // ros::Publisher reset_pub9 = nh_.advertise<std_msgs::Empty>("/bot9/reset", 1);
    std_msgs::Empty empty_msg;
    // Publish the empty message
    reset_pub3.publish(empty_msg);
    reset_pub6.publish(empty_msg);
    reset_pub7.publish(empty_msg);
    // reset_pub9.publish(empty_msg);

    ros::Subscriber odom_sub3 = nh_.subscribe("/bot3/odom", 10, odomCallback_3);
    ros::Subscriber odom_sub6 = nh_.subscribe("/bot6/odom", 10, odomCallback_6);
    ros::Subscriber odom_sub7 = nh_.subscribe("/bot7/odom", 10, odomCallback_7);
    // ros::Subscriber odom_sub9 = nh_.subscribe("/bot9/odom", 10, odomCallback_9);

    vel_pub_3 = nh_.advertise<geometry_msgs::Twist>("/bot3/cmd_vel", 10);
    vel_pub_6 = nh_.advertise<geometry_msgs::Twist>("/bot6/cmd_vel", 10);
    vel_pub_7 = nh_.advertise<geometry_msgs::Twist>("/bot7/cmd_vel", 10);
    // vel_pub_9 = nh_.advertise<geometry_msgs::Twist>("/bot9/cmd_vel", 10);

    ros::Rate loop_rate(10);
    int c = 0;
    while (ros::ok()) 
    {
        ros::spinOnce();
        // Your code here
        if(c > 101) {
            break;
        } else if(c > 100) {
            linear_angular_vel_.linear.x = 0.0;
            linear_angular_vel_.angular.z = 0.0;
            // vel_pub_9.publish(linear_angular_vel_);
            vel_pub_3.publish(linear_angular_vel_);
            vel_pub_7.publish(linear_angular_vel_);
            vel_pub_6.publish(linear_angular_vel_);
            ROS_INFO("%s", "...Finish...");
            loop_rate.sleep();
        } else if(c < 20) {
            linear_angular_vel_.linear.x = 0.1;
            linear_angular_vel_.angular.z = 0.0;
            // vel_pub_9.publish(linear_angular_vel_);
            vel_pub_6.publish(linear_angular_vel_);
            vel_pub_3.publish(linear_angular_vel_);
            vel_pub_7.publish(linear_angular_vel_);
            ROS_INFO("%s", "...Forward...");
        } else if(c < 40) {
            linear_angular_vel_.linear.x = -0.1;
            linear_angular_vel_.angular.z = 0.0;
            // vel_pub_9.publish(linear_angular_vel_);
            vel_pub_6.publish(linear_angular_vel_);
            vel_pub_3.publish(linear_angular_vel_);
            vel_pub_7.publish(linear_angular_vel_);
            ROS_INFO("%s", "...Backwards...");
        } else {
            linear_angular_vel_.linear.x = 0.0;
            linear_angular_vel_.angular.z = 3.1416/6;
            // vel_pub_9.publish(linear_angular_vel_);
            vel_pub_6.publish(linear_angular_vel_);
            vel_pub_3.publish(linear_angular_vel_);
            vel_pub_7.publish(linear_angular_vel_);
            ROS_INFO("%s", "...Rotating...");
        }
        // Sleep to control the loop rate
        loop_rate.sleep();
        ++c;
    }
    return 0;
}