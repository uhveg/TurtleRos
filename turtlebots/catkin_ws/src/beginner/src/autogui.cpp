#include "ros/ros.h"
// #include <fstream>
#include <cmath>
#include <GLFW/glfw3.h>
#include <vector>
#include "turtlebot_ctrl.h"

const float dcos = 0.05*std::cos(M_PI/8);
const float dsin = 0.05*std::sin(M_PI/8);

struct color {
    GLfloat r;
    GLfloat g;
    GLfloat b;
};

color color_tbots[] = {
    {136.0/255.0, 183.0/255.0, 181.0/255.0},
    {167.0/255.0, 202.0/255.0, 177.0/255.0},
    {65.0/255.0, 97.0/255.0, 101.0/255.0},
    {186.0/255.0, 149.0/255.0, 147.0/255.0}
};

void drawRobot(Turtlebot_Ctrl &tbot) {
    tbot.pushPosition();
    /* Draw the updated line */
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < tbot.positions.size(); i++) {
        glVertex2f(tbot.positions[i].x/2, tbot.positions[i].y/2);
    }

    float tcos = std::cos(tbot.pos.theta);
    float tsin = std::sin(tbot.pos.theta);

    glVertex2f(tbot.pos.x/2 - dcos*tcos - dsin*tsin, tbot.pos.y/2 - dcos*tsin + dsin*tcos);
    glVertex2f(tbot.pos.x/2 - dcos*tcos + dsin*tsin, tbot.pos.y/2 - dcos*tsin - dsin*tcos);
    glVertex2f(tbot.pos.x/2, tbot.pos.y/2);
    glEnd();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "GUI");
    ros::NodeHandle nh_;

    Turtlebot_Ctrl tbots[] = {
        Turtlebot_Ctrl(nh_, "bot3", -0.9, 0, M_PI_2, true),
        Turtlebot_Ctrl(nh_, "bot6", -0.3, 0, M_PI_2, true),
        Turtlebot_Ctrl(nh_, "bot7", 0.9, 0, M_PI_2, true),
        Turtlebot_Ctrl(nh_, "bot9", 0.3, 0, M_PI_2, true)
    };
    ros::Duration(0.5).sleep();
    for(int i=0; i<4;i++) {
        if(tbots[i].correctInitialized) {
            ROS_INFO("¡¡Bot%d correct initialized!!", i);
        } else {
            ROS_INFO("##Bot%d bad initialized##", i);
            return -1;
        }

    }

    // tbots[0].link(&tbots[1], 1.0);
    // tbots[0].link(&tbots[2], std::sqrt(2.0));
    // tbots[0].link(&tbots[3], 1.0);
    
    // tbots[1].link(&tbots[0], 1.0);
    // tbots[1].link(&tbots[2], 1.0);
    // tbots[1].link(&tbots[3], std::sqrt(2.0));

    // tbots[2].link(&tbots[0], std::sqrt(2.0));
    // tbots[2].link(&tbots[1], 1.0);
    // tbots[2].link(&tbots[3], 1.0);

    // tbots[3].link(&tbots[0], 1.0);
    // tbots[3].link(&tbots[1], std::sqrt(2.0));
    // tbots[3].link(&tbots[2], 1.0);

    ros::Rate loop_rate(60);

    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(640, 480, "Turtlebots", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    /* Loop until the user closes the window */
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    ros::Duration(2).sleep();
    while (ros::ok()) // !glfwWindowShouldClose(window) && 
    {
        ros::spinOnce();
        glClear(GL_COLOR_BUFFER_BIT);

        double sgd_area = 0.0;
        for(int i=0; i<4;i++) {
            int ip1 = (i + 1)%4;
            sgd_area += (tbots[i].posB.x*tbots[ip1].posB.y - tbots[ip1].posB.x*tbots[i].posB.y);
        }
        sgd_area *= 0.5;
        
        double dfx, dfy;
        for(int i=0; i<4; i++) {
            glColor3f(color_tbots[i].r, color_tbots[i].g, color_tbots[i].b);
            drawRobot(tbots[i]);
            
            Turtlebot_Ctrl *t_prev = &tbots[(i-1)%4];
            Turtlebot_Ctrl *t_next = &tbots[(i+1)%4];

            dfx = t_next->posB.y - t_prev->posB.y;
            dfx = -t_next->posB.x + t_prev->posB.x;
            tbots[i].setControl();
            tbots[i].setVelocity();
        }
        // for(int i=0; i<4; i++) {
            
        // }   
        
        glfwSwapBuffers(window);
        glfwPollEvents();
        // ROS_INFO("%s %d", "Size of positions: ", static_cast<int>(t0.positions.size()));
        loop_rate.sleep();
    }


    glfwTerminate();
    return 0;
}