#!/usr/bin/env python3
import serial
import time
from pynput import keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

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
    return key_pwm_commands.get(key)

# Send PWM commands to the serial port
def send_pwm_command(ser_port, key):
    command = get_pwm_command(key)
    if command:
        time.sleep(0.3)
        ser_port.flush()
        ser_port.write(command.encode())
        ser_port.flush()
    else:
        print("Invalid key.")

# Key press event handler
def on_press(key, ser_port):
    try:
        k = key.char  # single-char keys
        if k in ['w', 's', 'a', 'd', 'p']:
            send_pwm_command(ser_port, k)
    except AttributeError:
        print('Please enter the specified key only')

# Combined Node for encoder publishing and velocity subscription
class RobotController(Node):
    def __init__(self, ser_port):
        super().__init__('robot_controller')
        
        # Store serial port
        self.ser_port = ser_port
        
        # Previous velocity values for change detection
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0
        
        # Create publisher for encoder data
        self.publisher_ = self.create_publisher(String, '/encoder_data', 10)
        
        # Create subscription to cmd_vel
        self.velocity_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )
        
        self.get_logger().info("Robot Controller Node Initialized")

    def velocity_callback(self, msg):
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Check if velocity has changed
        if (linear_x != self.prev_linear_x) or (angular_z != self.prev_angular_z):
            # Send stop command first
            send_pwm_command(self.ser_port, 'p')
            time.sleep(0.1)

        # Determine command based on velocities
        if linear_x > 0:
            send_pwm_command(self.ser_port, 'w')
        elif linear_x < 0:
            send_pwm_command(self.ser_port, 's')
        elif angular_z > 0:
            send_pwm_command(self.ser_port, 'a')
        elif angular_z < 0:
            send_pwm_command(self.ser_port, 'd')
        else:
            send_pwm_command(self.ser_port, 'p')

        # Update previous velocities
        self.prev_linear_x = linear_x
        self.prev_angular_z = angular_z

    def publish_encoder_data(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing encoder data: {data}")

# Main function
def main():
    # Initialize ROS 2
    rclpy.init()
    
    # Initialize serial connection
    ser_port = init_serial_connection()
    
    # Create the combined node
    node = RobotController(ser_port)
    
    # # Start the keyboard listener in a separate thread
    # listener = keyboard.Listener(on_press=lambda event: on_press(event, ser_port))
    # listener.start()

    try:
        while 1:
            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0)
            
            # Read encoder data from the serial port
            line = ser_port.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"Raw encoder data: {line}")
                node.publish_encoder_data(line)

    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        ser_port.close()
        node.destroy_node()
        rclpy.shutdown()
        print("Serial port closed and ROS 2 shutdown.")

if __name__ == '__main__':
    main()