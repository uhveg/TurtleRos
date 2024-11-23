# Turtlebot Connection Automation App

This project is a Flask-based application designed to automate the connection of multiple Turtlebot robots to a ROS master. It eliminates the need for manually accessing each Turtlebot via SSH and running the required ROS files. 

The app provides an easy-to-use web interface for connecting and managing Turtlebots, making it more efficient to control multiple robots in a ROS environment.

---

## Features

- Automatically connect to multiple Turtlebots via SSH.
- Execute ROS commands remotely on each Turtlebot.
- Verify the connection by checking active ROS topics.
- Trigger sound signals on successful connections.
- Safely terminate ROS launch files when ending a session.

---

## Prerequisites

1. **Python Requirements**:
   - Python 3.x
   - Required Python packages:
     ```bash
     pip install flask paramiko
     ```

2. **Other Tools**:
   - `firefox` browser (for automatically opening the web interface).
   - Docker (to manage ROS commands within containers).

3. **Turtlebot Setup**:
   - Turtlebots should have SSH enabled.
   - ROS environment configured with `roslaunch` and `rostopic` commands.

4. **System Requirements**:
   - The application assumes the ROS master is running on a specific IP address, which you configure through the app.

---

## Usage
### Starting the Application

1. Run the provided start_app.sh script:
    ```bash
    ./start_app.sh
    ```
    This will:
    - Launch the Flask application in the background.
    - Open the app's web interface in the default browser.

2. Once the app is running, you can interact with the web interface to connect, check, or terminate Turtlebots.

### Web Interface Routes
- Home (```/```): The main web interface.
- Ping (```/ping```):
   - Sends a connection request to a Turtlebot.
   - Parameters:
      - ```ip_bot```: The IP of the Turtlebot.
      - ```ip_master```: The IP of the ROS master.
- Check (```/check```):
   - Checks the status of multiple Turtlebots (currently under development).
- End (```/end```):
   - Terminates the ROS session on a specific Turtlebot.
   - Parameter:
      - ```ip_bot```: The IP of the Turtlebot.
### Stopping the Application
- Press ```CTRL+C``` in the terminal running the ```start_app.sh``` script. This will:
   - Kill the Flask application process.
   - Cleanly terminate any ongoing sessions.
---
### Bash Script Details (``start_app.sh``)
This script simplifies starting the app:

1. Runs the Flask application in the background.
2. Stores the app's process ID to facilitate termination later.
3. Opens the web interface in Firefox after a short delay.
4. Waits for a user interrupt (```CTRL+C```) to terminate the Flask app gracefully.
---
### Known Issues
- If a Turtlebot's ROS environment isn't set up correctly, the app might fail to connect.
- Ensure all IP addresses are reachable within the network.
- The ```check``` endpoint is currently a placeholder and requires implementation.
---
### Future Improvements
- Add functionality to dynamically discover Turtlebots on the network.
- Implement the ```/check``` route for multi-robot status monitoring.
- Add detailed logs for debugging failed connections.
---
### License
This project is licensed under the MIT License. See the LICENSE file for details.

---
### Author
- Ulises H-Venegas

   Robotics Engineer | ML Developer | Automation Specialist

Feel free to reach out for any questions or improvements!

---
