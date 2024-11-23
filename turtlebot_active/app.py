from flask import Flask, render_template, request, jsonify
import subprocess
import paramiko
import time

# Initialize the SSH client using Paramiko
client = paramiko.client.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # Automatically add host keys to known hosts

# Initialize the Flask application
app = Flask(__name__)
print(__name__)  # Print the name of the module, useful for debugging

# Route for the home page
@app.route("/")
def home():
    # Render the home page template
    return render_template('index.html')

# Route to handle the ping functionality
@app.route("/ping", methods=['POST'])
def ping():
    # Get the JSON data sent in the POST request
    data = request.get_json()
    bot_ip = data["ip_bot"]  # Extract the bot's IP address
    print(f"Received ping request for the IP: {bot_ip}!")
    
    # Check if the bot is reachable via ping
    active, stdout_ = ping(bot_ip)
    if active:
        # Connect to the bot via SSH using the given credentials
        client.connect(bot_ip, username="pi", password="turtlebot")
        # Execute the command to start the Turtlebot
        _stdin, _stdout_, _stderr = client.exec_command(f"./turtlebot.sh http://{data['ip_master']}:11311")
        _stderr = _stderr.read().decode()
        
        if len(_stderr) > 0:  # If there's an error in the SSH command
            active = False
            stdout_ = _stderr
        else:
            stdout_ = f"Bot: {bot_ip} connected to {data['ip_master']}"
            
            # Helper function to execute ROS commands inside Docker
            docker_command = lambda x: f'docker exec turtlebot_cont bash -c "source /opt/ros/kinetic/setup.bash && {x}"'
            topiclist = docker_command("rostopic list")  # Command to list ROS topics
            active = False
            
            # Wait up to 15 seconds to ensure the connection is established
            for _ in range(15):
                time.sleep(1)
                try:
                    # Check if the bot's sound topic exists
                    output = subprocess.check_output(topiclist, stderr=subprocess.STDOUT, universal_newlines=True, shell=True)
                    topic_list = output.strip().split('\n')
                    for topic in topic_list:
                        if f"bot{bot_ip[-1]}/sound" in topic:
                            active = True
                            # Publish a sound message to indicate success
                            playsound = docker_command(f"""rostopic pub /bot{bot_ip[-1]}/sound turtlebot3_msgs/Sound \\"{{'value': 1}}\\" -1""")
                            output = subprocess.check_output(playsound, stderr=subprocess.STDOUT, universal_newlines=True, shell=True)
                            break
                    if active:
                        break
                except subprocess.CalledProcessError as e:
                    print(f"Error: {e}")
            if not active:
                stdout_ = "Not Connected"
        client.close()  # Close the SSH connection
    
    # Return the result as JSON
    return jsonify({'result_text': f"{stdout_}", 'active': active})

# Route to handle bot checks
@app.route("/check")
def check():
    # Example route to handle checking multiple bots (currently incomplete)
    data = request.get_json()
    bot_ips = (data["ip_bot_0"], data["ip_bot_1"], data["ip_bot_2"], data["ip_bot_3"])

# Route to handle ending the bot's activity
@app.route("/end", methods=['POST'])
def end():
    # Get the JSON data sent in the POST request
    data = request.get_json()
    bot_ip = data["ip_bot"]  # Extract the bot's IP address
    print(f"Received end request for the IP: {bot_ip}!")
    
    # Check if the bot is reachable via ping
    active, stdout_ = ping(bot_ip)
    if active:
        # Helper function to execute ROS commands inside Docker
        docker_command = lambda x: f'docker exec turtlebot_cont bash -c "source /opt/ros/kinetic/setup.bash && {x}"'
        # Connect to the bot via SSH
        client.connect(bot_ip, username="pi", password="turtlebot")
        # Find the process running the ROS launch file
        _stdin, _stdout_, _stderr = client.exec_command(f"ps ux | grep 'roslaunch my_turtlebot robot.launch'")
        stdout_ = _stdout_.read().decode()
        lines = stdout_.split('\n')
        for line in lines:
            if 'roslaunch my_turtlebot robot.launch' in line:
                columns = line.split()
                pid = int(columns[1])  # Extract the PID of the process
                # Publish a sound message to indicate termination
                playsound = docker_command(f"""rostopic pub /bot{bot_ip[-1]}/sound turtlebot3_msgs/Sound \\"{{'value': 0}}\\" -1""")
                output = subprocess.check_output(playsound, stderr=subprocess.STDOUT, universal_newlines=True, shell=True)
                _stdin, _stdout_, _stderr = client.exec_command(f"kill {pid}")  # Kill the process
                break
        stdout_ = "Closed roslaunch node!"
        client.close()  # Close the SSH connection
    
    # Return the result as JSON
    return jsonify({'result_text': f"{stdout_}", 'active': active})

# Function to check if a host is reachable via ping
def ping(host):
    try:
        # Execute the ping command
        output = subprocess.check_output(['ping', '-c', '4', host])
        return True, output.decode('utf-8')
    except subprocess.CalledProcessError as e:
        # Return False if the ping fails
        return False, f"Ping failed: {e}"

# Run the Flask app if this script is executed directly
if __name__ == "__main__":
    # Change host to '0.0.0.0' to accept connections from any network
    app.run(host='127.0.0.1', port=5000)
