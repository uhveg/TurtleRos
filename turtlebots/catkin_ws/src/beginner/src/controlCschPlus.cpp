#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <GLFW/glfw3.h>
#include <vector>
#include <sys/time.h>
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
        Turtlebot_Ctrl(nh_, "bot3", -0.9, 0.0, 0.0, true),
        Turtlebot_Ctrl(nh_, "bot6", -0.3, 0, 0.0, true),
        Turtlebot_Ctrl(nh_, "bot7", 0.3, 0, 0.0, true),
        Turtlebot_Ctrl(nh_, "bot9", 0.9, 0, 0.0, true)
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

    tbots[0].link(&tbots[1], 0.8517626);
    tbots[1].link(&tbots[2], 0.8517626);
    tbots[2].link(&tbots[3], 0.8517626);
    tbots[3].link(&tbots[0], 0.8517626);
    
    // tbots[0].link(&tbots[2], 2.28817123);
    

    ros::Rate loop_rate(100);

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

    struct timeval startTime, endTime;
    gettimeofday(&startTime, NULL);

    std::vector<long> timestamps;


    while (!glfwWindowShouldClose(window) && ros::ok()) //  
    {
        ros::spinOnce();
        glClear(GL_COLOR_BUFFER_BIT);
        
        gettimeofday(&endTime, NULL);
        long startMicroseconds = startTime.tv_sec * 1000000 + startTime.tv_usec;
        long endMicroseconds = endTime.tv_sec * 1000000 + endTime.tv_usec;
        long elapsedTimeMicroseconds = endMicroseconds - startMicroseconds;
        timestamps.push_back(elapsedTimeMicroseconds);

        for(int i=0; i<4; i++) {
            glColor3f(color_tbots[i].r, color_tbots[i].g, color_tbots[i].b);
            drawRobot(tbots[i]);

            tbots[i].setControl();
        }

        double sgd_area = 0.0;
        for(int i=0; i<4;i++) {
            int ip1 = (i + 1)%4;
            sgd_area += (tbots[i].posB.x*tbots[ip1].posB.y - tbots[ip1].posB.x*tbots[i].posB.y);
        }
        sgd_area *= 0.5;

        double dfx, dfy;
        for(int i=0; i<4; i++) {
            Turtlebot_Ctrl *t_prev = &tbots[(i-1)%4];
            Turtlebot_Ctrl *t_next = &tbots[(i+1)%4];

            dfx = t_next->posB.y - t_prev->posB.y;
            dfy = -t_next->posB.x + t_prev->posB.x;
            tbots[i].setControlSGD(sgd_area, 1.0, dfx, dfy);
        }

        for(int i=0; i<4; i++) {
            tbots[i].setVelocity();
        }
        
        glfwSwapBuffers(window);
        glfwPollEvents();
        // ROS_INFO("%s %d", "Size of positions: ", static_cast<int>(t0.positions.size()));
        loop_rate.sleep();
    }

    glfwTerminate();
    std::ofstream outputFile("/root/logs/logfile_pos.txt");
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1; // Return an error code
    }
    for (size_t i=0; i < timestamps.size(); i++) {
        // Convert the duration to a numeric value in microseconds
        long stValue = timestamps[i];
        position t0_pos = tbots[0].positions[i];
        position t1_pos = tbots[1].positions[i];
        position t2_pos = tbots[2].positions[i];
        position t3_pos = tbots[3].positions[i];
        // Write the numeric value to the file
        outputFile << stValue << " " << t0_pos.x << " " << t0_pos.y << " " << t0_pos.theta << " "
                                     << t1_pos.x << " " << t1_pos.y << " " << t1_pos.theta << " "
                                     << t2_pos.x << " " << t2_pos.y << " " << t2_pos.theta << " "
                                     << t3_pos.x << " " << t3_pos.y << " " << t3_pos.theta << std::endl;
    }

    // Close the file
    outputFile.close();

    // Check if there was an error while closing the file
    if (outputFile.fail()) {
        std::cerr << "Error occurred while closing the file." << std::endl;
        return 1; // Return an error code
    }

    std::cout << "Positions have been written to the file." << std::endl;

    outputFile.open("/root/logs/logfile_vel.txt");
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1; // Return an error code
    }
    for (size_t i=0; i < timestamps.size(); i++) {
        // Convert the duration to a numeric value in microseconds
        long stValue = timestamps[i];
        velocity t0_vel = tbots[0].velocities[i];
        velocity t1_vel = tbots[1].velocities[i];
        velocity t2_vel = tbots[2].velocities[i];
        velocity t3_vel = tbots[3].velocities[i];
        // Write the numeric value to the file
        outputFile << stValue << " " << t0_vel.v << " " << t0_vel.w << " "
                                     << t1_vel.v << " " << t1_vel.w << " "
                                     << t2_vel.v << " " << t2_vel.w << " "
                                     << t3_vel.v << " " << t3_vel.w << std::endl;
    }

    // Close the file
    outputFile.close();

    // Check if there was an error while closing the file
    if (outputFile.fail()) {
        std::cerr << "Error occurred while closing the file." << std::endl;
        return 1; // Return an error code
    }

    std::cout << "Velocities have been written to the file." << std::endl;
    return 0;
}