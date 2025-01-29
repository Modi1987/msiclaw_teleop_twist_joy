import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from evdev import InputDevice, ecodes, list_devices
from rclpy.logging import get_logger
import subprocess
import time
from .utils import find_device_path

NODE_NAME = 'msiclaw_joy_teleop_manager'

class JoystickTeleopManager(Node):
    def __init__(self, target_device_name="Xbox360 Controller for Windows Mouse"):
        super().__init__(NODE_NAME)
        # Declare parameters
        self.declare_the_parameters()
        # Load parameters
        self.load_the_parameters()
        
        # Initialize subscriber
        self._subscriber = self.create_subscription(
            Joy, 
            'msiclaw_joy', 
            self.sub_callback, 
            10
        )

        self._message_time = None
        self._timer = self.create_timer(2, self.timer_callback)

    def declare_the_parameters(self):
        self.declare_parameters(
            namespace='', 
            parameters=[
                ("device_name", "Xbox360 Controller for Windows Mouse"),
                ("timeout_seconds", 1.0),
            ]
        )

    def load_the_parameters(self):
        self._device_name = self.get_parameter("device_name").value
        self._timeout_seconds = self.get_parameter("timeout_seconds").value

    def sub_callback(self, msg):
        self._message_time = time.time()

    def timer_callback(self):
        if self._message_time is None:
            self._message_time = time.time()
        delta_time = time.time() - self._message_time

        if delta_time > self._timeout_seconds:
            self.launch_node_if_ready()
            self._message_time = time.time()

    def launch_node_if_ready(self):
        # Check if the device is available
        logger = self.get_logger()
        target_device_name = self._device_name
        device_path = find_device_path(logger, target_device_name)

        if device_path is None:
            error_message = f"Target device name '{target_device_name}' is not present. Exiting."
            logger.error(error_message)
            return
        else:
            info_message = f"Target device name: '{target_device_name}' is found under the path: '{device_path}'"
            logger.info(info_message)

        # Launch the teleop joystick node
        try:
            subprocess.run(["ros2", "launch", "msiclaw_teleop_twist_joy", "joystick_teleop.launch.py"], check=True)
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to launch teleop joystick: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleopManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
