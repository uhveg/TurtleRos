#ifndef TRB_C
#define TRB_C

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include <algorithm>
#include <cstdio>
#include <vector>

class Turtlebot_Ctrl;

struct position {
    double x;
    double y;
    double theta;
};
struct neighbor {
    Turtlebot_Ctrl *bot;
    double k;
};
struct velocity {
    double v;
    double w;
};

class Turtlebot_Ctrl {

public:
    position pos, posB;
    std::vector<position> positions;
    std::vector<velocity> velocities;
    std::vector<neighbor> neighbors;
    bool correctInitialized;
    int id;

    Turtlebot_Ctrl(ros::NodeHandle &nh, const char* botName, double ox, double oy, double otheta, bool active);
    void setControl();
    void setControlSGD(double Z, double Z_star, double diff_x, double diff_y);
    void link(Turtlebot_Ctrl *t, float k);
    void setVelocity();
    void pushPosition();
    bool resetOdom();
    bool turnMotors(bool data);

private:
    double b_;
    position offset;
    velocity vel;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::Subscriber odom_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher turn_motors_pub_;
    ros::Publisher reset_pub_;

    geometry_msgs::Twist linear_angular_vel_;
    std_msgs::Bool motors_active;
};

class Common {
public:
    static double csch(double x);
    static void clamp(double &val, double min, double max);
};


#endif