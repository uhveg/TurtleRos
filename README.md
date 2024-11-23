# Turtlebot ros controller
## Avoid Issues
- Run the following command in a terminal to avoid issues with permissions of executables
```
chmod -R 777 TurtleRos/
```
- After install docker and docker-compose run the following command in a terminal to avoid issues with docker (restart the computer after this)
```
sudo usermod -aG docker $USER
```
- Install the following to avoid issue with *distutils* error (only occurs with python>=3.12)
```
sudo apt install python3-setuptools
```
## Requirements
- Docker 
```
sudo apt install docker.io
```
- Docker compose (v1 or v2) 
```
sudo apt install docker-compose-v2
```
- Python3 with flask and paramiko (install requirements.txt from the `turtlebot_active` project)
- Install the docker image **turtlebot**, either by
```
cd turtlebots
docker load -i turtlebots.tar
``` 

or 

```
cd turtlebots
docker build -t turtlebot:latest .
```

check if its correctly added by finding turtlebot:latest in
```
docker images
```
## Modify control
The folder `/turtlebots/catkin_ws` contains the ros controller package *beginner* (This is a shared folder, check /turtlebots/docker-compose.yaml).<br>
The file `/turtlebots/catkin_ws/src/beginner/src/turtlebot_ctrl.cpp` contains the implementation of the controller, in order to modify it, check functions *setControl()* and *setControlSGD()*, also check function *setVelocity()*.<br>
The file `/turtlebots/catkin_ws/src/beginner/src/controlCschPlusTri.cpp` has the control application, there we can adjust the link lengths and areas with their gains, also uses a log script to save in `/root/logs/` (this is a shared folder, check /turtlebots/docker-compose.yaml).




## Process

1. Power on the turtlebots
2. In a terminal window run the following command to start the docker container (leave the terminal window open)
```
docker compose up
```
3. In another terminal window inside the container (see **Extra**), run roscore (make sure to modify ROS_IP and ROS_MASTER_URI from the /turtlebots/docker-compose.yaml file before running roscore)
```
roscore
```
4. Use the `turtlebots_active` project to connect to them via SSH and run the ros archive (*turtlebot.sh*) this file in the home folder of turtlebots 3, 6, 7, 9.

5. In another terminal window inside the container (see **Extra**), and after modify the control (if needed), compile the project with
```
cd ~/catkin_ws/
catkin_make
```
6. Run the app 
```
source ~/catkin_ws/devel/setup.bash # probably not needed
rosrun beginner ControlCschPlusTri
```
7. To finish, run in another terminal (outside of the container)
```
docker compose down
```
## Extra
- There is a file in the catkin workspace folder called `hard_stop.sh`, run it in a terminal inside the container to stop all the motion of the robots by constantly sending zero velocities.
- To start a bash session inside the container run:
```
docker exec -it turtlebot_cont bash
```