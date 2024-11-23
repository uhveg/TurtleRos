#!/bin/bash

# Get directory path
SCRIPT_DIR=$(dirname "$0")
echo $SCRIPT_DIR
python3 "$SCRIPT_DIR/app.py" &
FLASK_APP_ID=$!

sleep 1

firefox http://127.0.0.1:5000

echo "Press CTRL+C to terminate the app"

trap 'kill $FLASK_APP_ID' INT
wait $FLASK_APP_ID
