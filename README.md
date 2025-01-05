
# msiclaw_teleop_twist_joy

This is a ROS2 humble package that for interacting with the msiclaw joystick to teleoperate the robot

# To setup the joystick with linux

Launch the following

```
ros2 launch msiclaw_teleop_twist_joy joy.launch.py
```

Then echo the following

```
ros2 topic echo /msiclaw_joy
```