#include "turtlebot_ctrl.h"

Turtlebot_Ctrl::Turtlebot_Ctrl(ros::NodeHandle &nh, const char* botName, double ox, double oy, double otheta, bool active)
{
    correctInitialized = false;
    size_t length = strlen(botName);
    std::sscanf(botName, "bot%d", &id);
    char odom_topic[50], reset_topic[50], motors_topic[50], vel_topic[50];
    sprintf(odom_topic, "/%s/odom", botName);
    sprintf(reset_topic, "/%s/reset", botName);
    sprintf(motors_topic, "/%s/motor_power", botName);
    sprintf(vel_topic, "/%s/cmd_vel", botName);

    reset_pub_ = nh.advertise<std_msgs::Empty>(reset_topic, 1);
    turn_motors_pub_ = nh.advertise<std_msgs::Bool>(motors_topic, 1);
    vel_pub_ = nh.advertise<geometry_msgs::Twist>(vel_topic, 10);
    int timels = 0;
    if(resetOdom() && turnMotors(active)) {
        correctInitialized = true;
    }
    odom_sub_ = nh.subscribe(odom_topic, 10, &Turtlebot_Ctrl::odomCallback, this);
    b_ = 0.05; // offset of the center to control a diferential robot
    offset.x = ox;
    offset.y = oy;
    offset.theta = otheta;
    pos.x = offset.x;pos.y = offset.y;pos.theta = offset.theta;
    posB.x = pos.x + b_*std::cos(pos.theta);
    posB.y = pos.y + b_*std::sin(pos.theta);
}

void Turtlebot_Ctrl::setControl()
{
    double alpha = 0.7;
    double delta = 0.1;
    double ux = 0.0, uy = 0.0;
    for(size_t i=0; i < neighbors.size();i++) {
        double k = neighbors[i].k;
        double lx = posB.x - neighbors[i].bot->posB.x;
        double ly = posB.y - neighbors[i].bot->posB.y;
        double norm_l = std::sqrt(lx*lx + ly*ly);
        double csch_ = Common::csch((norm_l - delta) / k);
        double w = alpha*(1 - (1/(k * norm_l))*csch_*csch_);
        ux -= w*lx;
        uy -= w*ly;
        // ROS_INFO("%d-%d: %f, %f", id, neighbors[i].bot->id, neighbors[i].bot->posB.x, neighbors[i].bot->posB.y);
    }
    vel.v = (std::cos(pos.theta)*ux + std::sin(pos.theta)*uy);
    vel.w = (-std::sin(pos.theta)*ux/b_ + std::cos(pos.theta)*uy/b_);
}

void Turtlebot_Ctrl::setControlSGD(double Z, double Z_star, double diff_x, double diff_y) {
    // before calling this function, it is necessary to call setControl() once
    double K = 10.0;
    double ux = 0.0, uy = 0.0;
    ux = 0.5 * K * (Z - Z_star) * diff_x;
    uy = 0.5 * K * (Z - Z_star) * diff_y;
    vel.v -= (std::cos(pos.theta)*ux + std::sin(pos.theta)*uy);
    vel.w -= (-std::sin(pos.theta)*ux/b_ + std::cos(pos.theta)*uy/b_);
}

void Turtlebot_Ctrl::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Extract data from the Odometry message
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    // double z = msg->pose.pose.position.z;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double yaw = std::atan2(2 * (qx * qy + qw * qz), 1 - 2*(qy*qy + qz*qz));
    pos.theta = yaw + offset.theta;
    pos.x = std::cos(offset.theta)*x - std::sin(offset.theta)*y + offset.x;
    pos.y = std::sin(offset.theta)*x + std::cos(offset.theta)*y + offset.y;
    posB.x = pos.x + b_*std::cos(pos.theta);
    posB.y = pos.y + b_*std::sin(pos.theta);
    // if(!correctInitialized) {
    //     correctInitialized = true;
    //     ROS_INFO("%s%d!", "Odometry working for bot", id);
    // }
}

void Turtlebot_Ctrl::pushPosition() {
    positions.push_back(pos);
}

bool Turtlebot_Ctrl::resetOdom() {
    ROS_INFO("%s%d...", "Trying to reset odom for bot", id);
    int timels = 0;
    while (reset_pub_.getNumSubscribers() == 0) {
        if(timels > 6)
            return false;
        ros::Duration(0.5).sleep();
        timels++;
    }
    reset_pub_.publish(std_msgs::Empty());
    ROS_INFO("%s%d!", "Odometry reset for bot", id);
    return true;
}

bool Turtlebot_Ctrl::turnMotors(bool data) {
    motors_active.data = data;
    ROS_INFO("%s%d...", "Trying to deactivate motors for bot", id);
    int timels = 0;
    while (turn_motors_pub_.getNumSubscribers() == 0) {
        if(timels > 6)
            return false;
        ros::Duration(0.5).sleep();
        timels++;
    }
    turn_motors_pub_.publish(motors_active);
    ROS_INFO("%s%d!", "Motors activated for bot", id);
    return true;
}

void Turtlebot_Ctrl::setVelocity() {
    // adjust the following gains to not be always in the saturation zone
    vel.v *= 0.5;
    vel.w *= 0.5;
    
    // Saturate the velocities
    Common::clamp(vel.v, -0.2, 0.2);
    Common::clamp(vel.w, -0.9, 0.9);
    
    // assign the values to the ros message
    linear_angular_vel_.linear.x = vel.v;
    linear_angular_vel_.angular.z = vel.w;
    
    // save the info for the logs
    velocities.push_back(vel);
    
    // publish the velocities message
    vel_pub_.publish(linear_angular_vel_);
    
    // reset velocity variables 
    vel.v = 0;vel.w = 0;
}

void Turtlebot_Ctrl::link(Turtlebot_Ctrl *t, float k)
{
    neighbor n;
    n.bot = t;
    n.k = k;
    neighbors.push_back(n);
    neighbor m;
    m.bot = this;
    m.k = k;
    t->neighbors.push_back(m);
}

double Common::csch(double x) {
    return 1.0 / std::sinh(x);
}

void Common::clamp(double &val, double min, double max) {
    val = val < min? min : val > max? max : val;
}
