#! /usr/bin/env python3
# make sure to: pip install evdev
# make sure to: sudo usermod -aG input $USER
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from evdev import InputDevice, ecodes, list_devices
from rclpy.logging import get_logger

NODE_NAME = 'msiclaw_joystick_publisher'

def find_device_path(logger, target_name):
    devices = [InputDevice(path) for path in list_devices()]
    logger.info("Searching devices")
    for device in devices:
        if target_name in device.name:
            info_message = f"found the joystick device with target_name: \'{target_name}\'"
            logger.info(info_message)
            return device.path
    return None

class JoystickPublisher(Node):
    def __init__(self, device_path='/dev/input/event8'):
        super().__init__(NODE_NAME)
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.joy_msg = Joy()

        try:
            self.device = InputDevice(device_path)
            self.get_logger().info(f'Connected to device: {self.device.name}')
        except Exception as e:
            self.get_logger().error(f'Failed to open device {device_path}: {e}')
            raise

        # Timer for polling joystick events
        self.timer = self.create_timer(0.01, self.poll_device)
        self.axes = [0.0, 0.0]  # Assuming REL_X and REL_Y as axes
        self.buttons = []       # Add buttons here if needed
    
    
    def poll_device(self):
        try:
            for event in self.device.read():
                if event.type == ecodes.EV_REL:
                    if event.code == ecodes.REL_X:
                        self.axes[0] = float(event.value)
                    elif event.code == ecodes.REL_Y:
                        self.axes[1] = float(event.value)
                elif event.type == ecodes.EV_KEY:
                    # Handle button presses here if necessary
                    pass
                # Publish Joy message on SYN_REPORT
                if event.type == ecodes.SYN_REPORT:
                    self.joy_msg.header.stamp = self.get_clock().now().to_msg()
                    self.joy_msg.axes = self.axes
                    self.joy_msg.buttons = self.buttons
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

            # Timeout threshold in seconds (40 milliseconds = 0.04 seconds)
            timeout_threshold = rclpy.duration.Duration(seconds=0.04)

            if interval > timeout_threshold:
                self.joy_msg.axes = [0., 0.]
                self.joy_msg.buttons = []
            # publish the message
            self.publisher_.publish(self.joy_msg)

def main(args=None):
    rclpy.init(args=args)
    logger = get_logger(NODE_NAME)

    target_device_name = "Xbox360 Controller for Windows Mouse"
    device_path = find_device_path(logger, target_device_name)

    if target_device_name is None:
        error_message = f"Target device name \'{target_device_name}\' is not present. Exiting."
        logger.error(error_message)
        return
    else:
        error_message = f"Target device name: \'{target_device_name}\' is found under the path: \'{device_path}\'"
        logger.info(error_message)

    node = JoystickPublisher(device_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
