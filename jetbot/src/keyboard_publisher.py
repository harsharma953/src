#!/usr/bin/env python3

import serial
import time
from pynput import keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Initialize the serial connection
def init_serial_connection(port='/dev/ttyACM0', baudrate=115200, timeout=1):
    ser_port = serial.Serial(port, baudrate=baudrate, timeout=timeout)
    if not ser_port.is_open:
        ser_port.open()
    print("Serial connection initialized.")
    return ser_port

# Function to map keys to PWM commands
def get_pwm_command(key):
    key_pwm_commands = {
        'w': 'w 100 100 \n',  # Forward
        's': 's -100 -100 \n', # Backward
        'a': 'a -100 100 \n',   # Left turn
        'd': 'd 100 -100 \n',   # Right turn
        'p': 'p 0 0 \n'
    }
    # print(key_pwm_commands.get(key))
    return key_pwm_commands.get(key)

# Send PWM commands to the serial port
def send_pwm_command(ser_port, key):
    command = get_pwm_command(key)
    # print(command)
    if command:
        time.sleep(0.3)
        ser_port.flush()
        ser_port.write(command.encode())
        ser_port.flush()
        # print(f"Sent command: {command.strip()}")
    else:
        print("Invalid key.")

# Key press event handler
def on_press(key, ser_port):
    try:
        k = key.char  # single-char keys
        if k in ['w', 's', 'a', 'd', 'p']:
            # print(f'User input is {k}. Sending to serial port')
            send_pwm_command(ser_port, k)
    except AttributeError:
        print('Please enter the specified key only')

# Node to publish encoder data
class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.publisher_ = self.create_publisher(String, '/encoder_data', 10)
        self.get_logger().info("Encoder Publisher Node Initialized")

    def publish_encoder_data(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing encoder data: {data}")

# Main function
def main():
    # Initialize ROS 2
    rclpy.init()
    node = EncoderPublisher()

    # Initialize serial connection
    ser_port = init_serial_connection()

    # Start the keyboard listener in a separate thread
    listener = keyboard.Listener(on_press=lambda event: on_press(event, ser_port))
    listener.start()

    try:
        while listener.is_alive():
            # Read encoder data from the serial port
            # time.sleep(1)
            line = ser_port.readline().decode('utf-8', errors='ignore').strip()
            
            print(f"Raw encoder data: {line}")

            node.publish_encoder_data(line)  # Publish encoder data to ROS 2 topic
            # time.sleep(0.1)w
    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        listener.stop()
        ser_port.close()
        node.destroy_node()
        rclpy.shutdown()
        print("Serial port closed and ROS 2 shutdown.")

if __name__ == '__main__':
    main()
