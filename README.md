
# msiclaw_teleop_twist_joy

This is a ROS2 humble package that for interacting with the msiclaw joystick to teleoperate the robot

# To compile

```
git clone ..the project to ur source folder....
cd ..to ur work space...
colcon build
```

# To setup the joystick with linux as a service

To setup a linux service that will handle the Msiclaw joystick and publish as ROS2 topic use the this setup script, [setup_joystick_as_linux_service.sh](bash/setup_joystick_as_linux_service.sh) as in the following:

```
cd bash
./setup_joystick_as_linux_service.sh --ROS_DOMAIN_ID YOUR_ROS2_DOMAIN_ID
```


# If you prefer to launch the node nanually from terminal

Launch the following

```
ros2 launch msiclaw_teleop_twist_joy joy.launch.py
```

Then echo the following

```
ros2 topic echo /handheld_joy

ros2 topic echo /input/cmd_vel_teleop_handheld_joy
```