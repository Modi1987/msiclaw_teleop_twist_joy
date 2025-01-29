#!/bin/bash

SERVICE_FILE="msiclaw_teleop_manager.service"
SYSTEMD_PATH="/etc/systemd/system"

ROS_DOMAIN_ID=""

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --ROS_DOMAIN_ID) ROS_DOMAIN_ID="$2"; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Check if required arguments are provided
if [[ -z "$ROS_DOMAIN_ID" ]]; then
    echo "Usage: $0 --ROS_DOMAIN_ID <ROS_DOMAIN_ID>"
    exit 1
fi

# Check if ROS_DISTRO is set
if [[ -z "$ROS_DISTRO" ]]; then
    echo "Error: ROS_DISTRO is not set. Please source your ROS 2 environment (e.g., source /opt/ros/foxy/setup.bash)."
    exit 1
fi

# installing evdev
sudo apt install python3-evdev

# fix issue user is not in input
sudo usermod -aG input $USER

# Install teleop_twist_joy
echo "Installing teleop_twist_joy for ROS2 distro: ${ROS_DISTRO}..."
sudo apt-get install -y ros-${ROS_DISTRO}-teleop-twist-joy

# Set up the msiclaw_teleop_twist_joy as a service
echo "Specified ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

# Function to find the workspace directory dynamically
find_workspace_dir() {
    local dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"  # Start from the script's directory
    while [[ "$dir" != "/" ]]; do
        if [[ -d "$dir/src" ]]; then
            echo "$dir"  # Found the workspace directory
            return 0
        fi
        dir="$(dirname "$dir")"  # Move up one directory
    done
    echo "Error: Could not find workspace directory containing a 'src' folder!" >&2
    exit 1
}

# Call the function and assign the result to WORKSPACE_SOURCE_DIR
WORKSPACE_SOURCE_DIR=$(find_workspace_dir)
echo "Detected workspace directory: ${WORKSPACE_SOURCE_DIR}"

if [ -f "$SYSTEMD_PATH/$SERVICE_FILE" ]; then
    echo "Service file $SYSTEMD_PATH/$SERVICE_FILE already exists. It will be removed and reinstalled!"
    sudo rm -f "$SYSTEMD_PATH/$SERVICE_FILE"
fi

echo "Creating the systemd service file for joystick_teleop_manager..."

# Write the service file content
sudo bash -c "cat > $SYSTEMD_PATH/$SERVICE_FILE" <<EOL
[Unit]
Description=MsiClaw Joystick Teleop Manager ROS2 Node
After=network.target

[Service]
ExecStart=/bin/bash -c "source $HOME/.bashrc && source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WORKSPACE_SOURCE_DIR}/install/setup.bash && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && ros2 launch msiclaw_teleop_twist_joy msiclaw_joy_teleop_manager.launch.py"
WorkingDirectory=${WORKSPACE_SOURCE_DIR}
Restart=always
User=$USER
Group=$USER

[Install]
WantedBy=multi-user.target
EOL

echo "Service file created at $SYSTEMD_PATH/$SERVICE_FILE"

# Reload systemd to pick up the new service
echo "Reloading systemd..."
sudo systemctl daemon-reload

# Enable the service to start on boot
echo "Enabling ${SERVICE_FILE} service..."
sudo systemctl enable ${SERVICE_FILE}

# Start the service
echo "Starting ${SERVICE_FILE} service..."
sudo systemctl start ${SERVICE_FILE}

# Check service status
echo "Checking ${SERVICE_FILE} status..."
sudo systemctl status ${SERVICE_FILE}

echo "Setup complete. The service is now running!"
