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

position turtlebot0 = {0,0,0}, turtlebot1 = {0.1,0.67,0};
position turtlebot0_offset = {0,0,0}, turtlebot1_offset = {0.1,0.67,0};


void odomCallback_0(const nav_msgs::Odometry::ConstPtr& msg) {
    // Extract data from the Odometry message
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    // double z = msg->pose.pose.position.z;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double yaw = std::atan2(2 * (qx * qy + qw * qz), 1 - 2*(qy*qy + qz*qz));
    turtlebot0.x = turtlebot0_offset.x + x;
    turtlebot0.y = turtlebot0_offset.y + y;
    turtlebot0.theta = turtlebot0_offset.theta + yaw;
    // ROS_INFO("%s %f", "YAW: ", yaw);8
}
void odomCallback_1(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    // double z = msg->pose.pose.position.z;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double yaw = std::atan2(2 * (qx * qy + qw * qz), 1 - 2*(qy*qy + qz*qz));
    turtlebot1.x = turtlebot1_offset.x + x;
    turtlebot1.y = turtlebot1_offset.y + y;
    turtlebot1.theta = turtlebot1_offset.theta + yaw;
}

double csch(double x) {
    return 1.0 / std::sinh(x);
}

void clamp(double &val, double min, double max) {
    val = val < min? min : val > max? max : val;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower");
    ros::NodeHandle nh_;

    ros::Duration(1.0).sleep();

    ros::Publisher vel_pub;
    geometry_msgs::Twist linear_angular_vel_;

    ros::Publisher reset_pub3 = nh_.advertise<std_msgs::Empty>("/bot3/reset", 1);
    ros::Publisher reset_pub6 = nh_.advertise<std_msgs::Empty>("/bot6/reset", 1);
    std_msgs::Empty empty_msg;
    std_msgs::Bool bool_msg;
    bool_msg.data = false;
    // Publish the empty message
    reset_pub3.publish(empty_msg);
    reset_pub6.publish(empty_msg);

    ros::Publisher turn_motors_3 = nh_.advertise<std_msgs::Bool>("/bot3/motor_power", 1);
    ros::Publisher turn_motors_6 = nh_.advertise<std_msgs::Bool>("/bot6/motor_power", 1);
    // bool_msg.data = true;
    // turn_motors_3.publish(bool_msg);
    turn_motors_6.publish(bool_msg);

    ros::Subscriber odom_sub3 = nh_.subscribe("/bot3/odom", 30, odomCallback_0);
    ros::Subscriber odom_sub6 = nh_.subscribe("/bot6/odom", 30, odomCallback_1);
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/bot3/cmd_vel", 10);

    ros::Rate loop_rate(30);

    double dx, dy;
    double alpha = 1.0, K = 0.5, delta = 0.1;
    double norm_l_ij, w_ij;
    double csch_aux;
    double b = 0.03;

    while (ros::ok()) 
    {
        ros::spinOnce();
        // Your code here

        double tx0 = turtlebot0.x + b*std::cos(turtlebot0.theta);
        double ty0 = turtlebot0.y + b*std::sin(turtlebot0.theta);

        double tx1 = turtlebot1.x + b*std::cos(turtlebot1.theta);
        double ty1 = turtlebot1.y + b*std::sin(turtlebot1.theta);

        dx = tx0 - tx1;
        dy = ty0 - ty1;

        norm_l_ij = std::sqrt(dx*dx + dy*dy);
        csch_aux = csch((norm_l_ij - delta)/K);
        w_ij = alpha*(1 - (1/(K*norm_l_ij))* csch_aux * csch_aux );

        dx *= -w_ij;
        dy *= -w_ij;

        double linear_ = std::cos(turtlebot0.theta)*dx + std::sin(turtlebot0.theta)*dy;
        double angular_ = -std::sin(turtlebot0.theta)*dx/b + std::cos(turtlebot0.theta)*dy/b;

        clamp(linear_, -0.2, 0.2);
        clamp(angular_, -0.9, 0.9);

        linear_angular_vel_.linear.x = linear_;
        linear_angular_vel_.angular.z = angular_;

        vel_pub.publish(linear_angular_vel_);
        ROS_INFO("%s %f, %f", "Velocity: ", linear_angular_vel_.linear.x, linear_angular_vel_.angular.z);
        // ROS_INFO("%s %f", "Rotation: ", turtlebot0.theta);
        // Sleep to control the loop rate
        loop_rate.sleep();
    }
    return 0;
}