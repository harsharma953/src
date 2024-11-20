#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from std_msgs.msg import String 
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class RobotOdometryCalculator:
    def __init__(self):
        # Robot parameters
        self.wheel_radius = 0.05  # meters
        self.wheel_base = 0.55    # distance between wheels
        
        # Position state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Encoder parameters
        self.ticks_per_revolution = 355
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_revolution
        
        # Previous state
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = None
        
    def update_odometry(self, left_ticks, right_ticks, current_time):
        # Initialize previous state if this is the first update
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            return self.x, self.y, self.theta
        
        try:
            # Calculate time difference
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            
            # Protect against zero or negative time differences
            if dt <= 0:
                return self.x, self.y, self.theta
                
            # Calculate distance traveled by each wheel
            left_ticks_diff = left_ticks - self.prev_left_ticks
            right_ticks_diff = right_ticks - self.prev_right_ticks
            
            # Print encoder ticks for debugging
            print("\nEncoder Ticks:")
            print(f"Left Ticks: {left_ticks}, Right Ticks: {right_ticks}")
            print(f"Left Diff: {left_ticks_diff}, Right Diff: {right_ticks_diff}")
            
            left_distance = left_ticks_diff * self.meters_per_tick
            right_distance = right_ticks_diff * self.meters_per_tick
            
            # Calculate center distance and rotation
            # Calculate center distance and rotation
            center_distance = (left_distance + right_distance) / 2.0
            # Fix: Divide by 2 to correct the rotation calculation
            rotation = (right_distance - left_distance) / (3.0 * self.wheel_base)
            
            # Store old position for printing change
            old_x = self.x
            old_y = self.y
            old_theta = self.theta
            
            # Update pose
            if abs(rotation) < 0.0001:  # Moving straight
                self.x += center_distance * math.cos(self.theta)
                self.y += center_distance * math.sin(self.theta)
            else:  # Moving in an arc
                radius = center_distance / rotation
                self.x += radius * (math.sin(self.theta + rotation) - math.sin(self.theta))
                self.y -= radius * (math.cos(self.theta + rotation) - math.cos(self.theta))
            
            self.theta += rotation
            # Normalize angle to [-π, π]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # Print odometry information
            print("\nOdometry Position:")
            print(f"X: {self.x:.4f} meters")
            print(f"Y: {self.y:.4f} meters")
            print(f"Theta: {math.degrees(self.theta):.2f} degrees")
            
            # Print changes
            print("\nPosition Changes:")
            print(f"ΔX: {(self.x - old_x):.4f} meters")
            print(f"ΔY: {(self.y - old_y):.4f} meters")
            print(f"ΔTheta: {math.degrees(self.theta - old_theta):.2f} degrees")
            print("-" * 50)
            
            # Update previous values
            self.prev_time = current_time
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            
        except Exception as e:
            print(f"Error in odometry update: {e}")
            # Return current state without updates in case of error
            
        return self.x, self.y, self.theta
        
    def create_odometry_message(self, x, y, theta, current_time):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        # Set position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        quat = self.quaternion_from_euler(0.0, 0.0, theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Set velocity
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt > 0:
                # Linear velocity
                odom.twist.twist.linear.x = (x - self.x) / dt
                odom.twist.twist.linear.y = (y - self.y) / dt
                # Angular velocity
                angle_diff = theta - self.theta
                # Normalize angle difference
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                odom.twist.twist.angular.z = angle_diff / dt
        
        return odom
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('odometry_publisher')
    
    odom_publisher = node.create_publisher(Odometry, '/odom', 10)
    tf_broadcaster = TransformBroadcaster(node)
    
    odometry_calculator = RobotOdometryCalculator()
    
    def encoder_callback(msg):
        current_time = node.get_clock().now()
        
        # Parse encoder data
        try:
            if msg.data == 'Null':
                return
            
            parts = msg.data.split(' ')
            if len(parts) < 2:
                return
                
            left_ticks, right_ticks = map(float, parts[:2])
            
            # Update odometry
            x, y, theta = odometry_calculator.update_odometry(left_ticks, right_ticks, current_time)
            
            # Publish odometry message
            odom_msg = odometry_calculator.create_odometry_message(x, y, theta, current_time)
            odom_publisher.publish(odom_msg)
            
            # Publish transform
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            quat = odometry_calculator.quaternion_from_euler(0.0, 0.0, theta)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            node.get_logger().error(f'Error in encoder callback: {e}')
    
    subscription = node.create_subscription(
        String,
        '/encoder_data',
        encoder_callback,
        10
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()