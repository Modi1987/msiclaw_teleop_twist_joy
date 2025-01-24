#!/bin/bash

SERVICE_FILE="msiclaw_teleop_manager.service"
SYSTEMD_PATH="/etc/systemd/system"

ROS2_DOMAIN_ID=""

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --ROS2_DOMAIN_ID) ROS2_DOMAIN_ID="$2"; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Check if required arguments are provided
if [[ -z "$ROS2_DOMAIN_ID" ]]; then
    echo "Usage: $0 --ROS2_DOMAIN_ID <ROS_DOMAIN_ID>"
    exit 1
fi

if [[ -z "$ROS_DISTRO" ]]; then
    echo "Error: ROS_DISTRO is not set. Please source your ROS 2 environment (e.g., source /opt/ros/foxy/setup.bash)."
    exit 1
fi

# install teleop_twist_joy
echo "ROS2 distro is: " ${ROS_DISTRO}
sudo apt-get install ros-${ROS_DISTRO}-teleop-twist-joy

# setup the msiclaw_teleop_twist_joy as a service
echo "specfied ROS2_DOMAIN_ID: " ${ROS2_DOMAIN_ID}

# start building
echo "Buidling the package msiclaw_teleop_twist_joy"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SOURCE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")")"
echo "current workspace is: ${WORKSPACE_SOURCE_DIR}"

if [ -f  $SYSTEMD_PATH/$SERVICE_FILE ]; then
    echo "Service file $SYSTEMD_PATH/$SERVICE_FILE already exists, will be removed and reinstalled!"
    sudo bash -c "rm -f $SYSTEMD_PATH/$SERVICE_FILE"
fi

echo "Creating the systemd service file for joystick_teleop_manager..."

# Write the service file content
sudo bash -c "cat > $SYSTEMD_PATH/$SERVICE_FILE" <<EOL
[Unit]
Description=MsiClaw Joystick Teleop Manager ROS2 Node
After=network.target

[Service]
ExecStart=/bin/bash -c "source /home/${USER}/.bashrc && source ${WORKSPACE_SOURCE_DIR}/install/setup.bash && export ROS2_DOMAIN_ID=${ROS2_DOMAIN_ID} && ros2 launch msiclaw_teleop_twist_joy msiclaw_joy_teleop_manager.launch.py"
WorkingDirectory=${WORKSPACE_SOURCE_DIR}
Restart=on-failure
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

echo "Setup complete. You can now start the service using: sudo systemctl start ${SERVICE_FILE}"
