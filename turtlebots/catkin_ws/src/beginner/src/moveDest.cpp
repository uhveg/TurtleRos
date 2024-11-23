#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include "nav_msgs/Odometry.h"
// #include <fstream>
#include <cmath>

struct position
{
    double x;
    double y;
    double theta;
};

position turtlebot = {0,0,0};


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Extract data from the Odometry message
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    // double z = msg->pose.pose.position.z;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double yaw = std::atan2(2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
    turtlebot.x = x;
    turtlebot.y = y;
    turtlebot.theta = yaw;
}

void clamp(double &val, double min, double max) {
    val = val < min? min : val > max? max : val;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower");
    ros::NodeHandle nh_;

    ros::Publisher vel_pub;
    geometry_msgs::Twist linear_angular_vel_;

    double alpha = 0.1, K = 0.5, delta = 0.1;
    double norm_l_ij, w_ij;
    double csch_aux;
    ros::Publisher reset_pub3 = nh_.advertise<std_msgs::Empty>("/bot3/reset", 1);
    std_msgs::Empty empty_msg;
    std_msgs::Bool bool_msg;
    // bool_msg.data = false;
    // Publish the empty message
    reset_pub3.publish(empty_msg);

    ros::Publisher turn_motors_3 = nh_.advertise<std_msgs::Bool>("/bot3/motor_power", 1);
    bool_msg.data = true;
    turn_motors_3.publish(bool_msg);


    ros::Subscriber odom_sub3 = nh_.subscribe("/bot3/odom", 30, odomCallback);
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/bot3/cmd_vel", 5);

    ros::Rate loop_rate(30);

    double dx, dy;
    double b = 0.08;

    while (ros::ok()) 
    {
        ros::spinOnce();

        double tx = turtlebot.x + b*std::cos(turtlebot.theta);
        double ty = turtlebot.y + b*std::sin(turtlebot.theta);

        dx = (1 - tx) * 0.2;
        dy = (1 - ty) * 0.2;

        double linear_ = std::cos(turtlebot.theta)*dx + std::sin(turtlebot.theta)*dy;
        double angular_ = -std::sin(turtlebot.theta)*dx/b + std::cos(turtlebot.theta)*dy/b;

        clamp(linear_, -0.2, 0.2);
        clamp(angular_, -0.9, 0.9);

        linear_angular_vel_.linear.x = linear_;
        linear_angular_vel_.angular.z = angular_;

        vel_pub.publish(linear_angular_vel_);
        ROS_INFO("%s %f, %f", "Velocity: ", linear_angular_vel_.linear.x, linear_angular_vel_.angular.z);
        // Sleep to control the loop rate
        loop_rate.sleep();
    }
    return 0;
}