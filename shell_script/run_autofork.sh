#!/bin/bash

# Print startup message
echo "Started the Autofork interface"

# Function to clean up background processes
cleanup() {
    echo ""
    echo "Closed the Autofork interface"
    # Kill all background processes started by this script
    kill $(jobs -p)
    # Forcefully terminate rosbridge_websocket to free port 9090
    pkill -9 -f "rosbridge_websocket --port 9090"
    exit 0
}

# Trap Ctrl+C (SIGINT) and call cleanup function
trap cleanup SIGINT

# Change to the workspace directory and source setup
cd ~/autofork_ws
source install/setup.bash

# Run ROS 2 processes in the background with output redirection
ros2 run rosbridge_server rosbridge_websocket --port 9090 > rosbridge.log 2>&1 &
ros2 run rosboard rosboard_node > rosboard.log 2>&1 &

# Change to the node server directory and run the server in the background
cd ~/autofork_ws/src/Robot_Panel/rosboard/html/js/server/
node server.js > server.log 2>&1 &

# Wait for all background processes to finish
wait