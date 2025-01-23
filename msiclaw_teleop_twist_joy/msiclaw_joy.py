#! /usr/bin/env python3
# make sure to: pip install evdev
# make sure to: sudo usermod -aG input $USER
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from evdev import InputDevice, ecodes
from rclpy.logging import get_logger
import copy
from .utils import find_device_path

NODE_NAME = 'msiclaw_joystick_publisher'

class JoystickPublisher(Node):
    def __init__(self, target_device_name = "Xbox360 Controller for Windows Mouse"):
        super().__init__(NODE_NAME)
        # declare parameters
        self.declare_the_parameters()
        # get paramters
        self.load_the_parameters()
        # get device
        logger = self.get_logger()
        device_path = find_device_path(logger, target_device_name)
        if target_device_name is None:
            error_message = f"Target device name \'{target_device_name}\' is not present. Exiting."
            logger.error(error_message)
            return None
        else:
            error_message = f"Target device name: \'{target_device_name}\' is found under the path: \'{device_path}\'"
            logger.info(error_message)

        # following emulating xbox controller
        self.joy_msg = Joy()
        self.axes_latest_received_raw_vlaues = [0.0]*6
        self.buttons_latest_received_raw_vlaues = [0]*13
        self.buttons_latest_received_raw_vlaues[5] = 1
        self.ZERO_AXES_VALUES = [0.0]*6
        self.ZERO_BUTTONS_VALUES = [0]*13
        self.assign_joy_values(self.ZERO_AXES_VALUES, self.ZERO_BUTTONS_VALUES)
        
        self.publisher_ = self.create_publisher(Joy, 'msiclaw_joy', 10)

        try:
            self.device = InputDevice(device_path)
            self.get_logger().info(f'Connected to device: {self.device.name}')
        except Exception as e:
            self.get_logger().error(f'Failed to open device {device_path}: {e}')
            raise

        # Timer for polling joystick events
        update_interval = 1.0 / self.autorepeat_rate
        self.timer = self.create_timer(update_interval, self.read_and_publish_routine)
    
    def assign_joy_values(self, axes, buttons):
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        normalized_axes = copy.copy(axes)
        N = len(axes)
        for i in range(N):
            normalized_axes[i] = axes[i] / self.max_axis_value
            if abs(normalized_axes[i]) < self.deadzone:
                normalized_axes[i] = 0.
        self.joy_msg.axes = normalized_axes
        self.joy_msg.buttons = copy.copy(buttons)
    
    def read_and_publish_routine(self):
        try:
            for event in self.device.read():
                if event.type == ecodes.EV_REL:
                    if event.code == ecodes.REL_X:
                        self.axes_latest_received_raw_vlaues[0] = -float(event.value)
                    elif event.code == ecodes.REL_Y:
                        self.axes_latest_received_raw_vlaues[1] = -float(event.value)
                elif event.type == ecodes.EV_KEY:
                    # Handle button presses here if necessary
                    pass
                # Assing joystick values on SYN_REPORT
                if event.type == ecodes.SYN_REPORT:
                    self.assign_joy_values(self.axes_latest_received_raw_vlaues, self.buttons_latest_received_raw_vlaues)
        except BlockingIOError:
            # Non-blocking read can sometimes have no data; skip
            pass
        except Exception as e:
            self.get_logger().error(f'Error reading device: {e}')
        finally:
            # check for timeout
            now = self.get_clock().now()
            last_msg_time = rclpy.time.Time.from_msg(self.joy_msg.header.stamp)
            interval = now - last_msg_time

            timeout_threshold = rclpy.duration.Duration(seconds=self.timout_seconds)

            if interval > timeout_threshold:
                self.assign_joy_values(self.ZERO_AXES_VALUES, self.ZERO_BUTTONS_VALUES)
        # publish the message
        self.publisher_.publish(self.joy_msg)

    def declare_the_parameters(self):
        self.declare_parameters(
            namespace = '', 
            parameters = [
                    ("device_name", "Xbox360 Controller for Windows Mouse"),
                    ("deadzone", 0.1),
                    ("autorepeat_rate", 20.0),
                    ("max_axis_value", 4.0),
                    ("timout_seconds", 0.001),
                ] 
            )
    
    def load_the_parameters(self):
        self.device_name = self.get_parameter("device_name").value
        self.deadzone = self.get_parameter("deadzone").value
        self.autorepeat_rate = self.get_parameter("autorepeat_rate").value
        self.max_axis_value = self.get_parameter("max_axis_value").value
        self.timout_seconds = self.get_parameter("timout_seconds").value


def main(args=None):
    rclpy.init(args=args)
    node = JoystickPublisher()
    if node is None:
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
