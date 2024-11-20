#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityTestPublisher(Node):
    def __init__(self):
        super().__init__('velocity_test_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(3.0, self.test_sequence)  # Run test sequence every 3 seconds
        self.test_step = 0
        self.get_logger().info('Velocity Test Publisher Started')

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published velocity - linear_x: {linear_x}, angular_z: {angular_z}')

    def test_sequence(self):
        # Test sequence of different movements
        if self.test_step == 0:
            # Forward
            self.publish_velocity(0.5, 0.0)
        elif self.test_step == 1:
            # Stop
            self.publish_velocity(0.0, 0.0)
        elif self.test_step == 2:
            # Backward
            self.publish_velocity(-0.5, 0.0)
        elif self.test_step == 3:
            # Stop
            self.publish_velocity(0.0, 0.0)
        elif self.test_step == 4:
            # Turn left
            self.publish_velocity(0.0, 0.5)
        elif self.test_step == 5:
            # Stop
            self.publish_velocity(0.0, 0.0)
        elif self.test_step == 6:
            # Turn right
            self.publish_velocity(0.0, -0.5)
        elif self.test_step == 7:
            # Stop
            self.publish_velocity(0.0, 0.0)

        # Reset the test sequence
        self.test_step = (self.test_step + 1) % 10

def main():
    rclpy.init()
    node = VelocityTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()