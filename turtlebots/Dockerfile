FROM ros:kinetic-ros-base

RUN apt-get update 

RUN apt-get install -y \
    ros-kinetic-dynamixel-sdk \
    ros-kinetic-turtlebot3-msgs \
    ros-kinetic-turtlebot3 \
    libglfw3 \
    libglfw3-dev \
    libgl1-mesa-glx
    
# Create a catkin workspace and set it as the default workspace
RUN mkdir -p ~/catkin_ws/src

# Activate the ROS environment using a subshell
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/kinetic/setup.bash && \
    cd ~/catkin_ws && \
    catkin_make

RUN source ~/catkin_ws/devel/setup.bash

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

CMD ["bash"]

