#!/bin/bash

# Start the containers with docker-compose up
docker compose up -d

# Sleep for a moment to allow the containers to start
sleep 5

# Get the container ID of the service you want to access
container_id=$(docker compose ps -q turtlebot)

# Check if the container ID is empty
if [ -z "$container_id" ]; then
  echo "Container not found. Exiting."
  exit 1
fi

# Open a new terminal and execute docker exec
gnome-terminal -- bash -c "docker exec -it $container_id bash"

# Open a new terminal and execute python webpage
./../turtlebot_active/turtleApp.sh
