from evdev import InputDevice, list_devices

def find_device_path(logger, target_name):
    devices = [InputDevice(path) for path in list_devices()]
    logger.info("Searching devices")
    for device in devices:
        if target_name in device.name:
            info_message = f"Found the joystick device with target_name: '{target_name}'"
            logger.info(info_message)
            return device.path
    logger.error(f"Device with target_name: '{target_name}' not found.")
    return None