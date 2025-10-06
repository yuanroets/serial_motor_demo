#!/usr/bin/env python3
"""
Simple Odometry Node for SLAM
=============================

Converts serial_motor_demo encoder data to odometry transforms
for SLAM. This publishes the odom->base_link transform that SLAM needs.

Based on differential drive kinematics.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from serial_motor_demo_msgs.msg import EncoderVals
from tf2_ros import TransformBroadcaster
import math
import time


class SimpleOdometry(Node):
    def __init__(self):
        super().__init__('simple_odometry')
        
        # Parameters (matching your actual hardware)
        self.declare_parameter('encoder_cpr', 1860)
        self.declare_parameter('wheel_separation', 0.170)  # Final optimized value through systematic testing
        self.declare_parameter('wheel_radius', 0.02569)     # 25.69mm radius (50mm diameter)
        
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Encoder subscriber
        self.encoder_sub = self.create_subscription(
            EncoderVals, 'encoder_vals', self.encoder_callback, 10)
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables
        self.last_left_enc = 0
        self.last_right_enc = 0
        self.last_time = time.time()
        self.first_reading = True
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.get_logger().info(f'Simple odometry started - CPR: {self.encoder_cpr}, '
                             f'wheel_sep: {self.wheel_separation}m, wheel_radius: {self.wheel_radius}m')
    
    def encoder_callback(self, msg):
        """Process encoder data and publish odometry"""
        current_time = time.time()
        
        if self.first_reading:
            # Initialize on first reading
            self.last_left_enc = msg.mot_1_enc_val   # Left motor is mot_1
            self.last_right_enc = msg.mot_2_enc_val  # Right motor is mot_2
            self.last_time = current_time
            self.first_reading = False
            return
        
        # Calculate encoder differences
        left_diff = msg.mot_1_enc_val - self.last_left_enc
        right_diff = msg.mot_2_enc_val - self.last_right_enc
        dt = current_time - self.last_time
        
        if dt <= 0:
            return
        
        # Convert encoder counts to wheel distances
        left_distance = (left_diff / self.encoder_cpr) * 2.0 * math.pi * self.wheel_radius
        right_distance = (right_diff / self.encoder_cpr) * 2.0 * math.pi * self.wheel_radius
        
        # Calculate robot motion
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation
        
        # Update pose
        self.x += distance * math.cos(self.theta + delta_theta/2.0)
        self.y += distance * math.sin(self.theta + delta_theta/2.0)
        self.theta += delta_theta
        
        # Calculate velocities
        linear_vel = distance / dt
        angular_vel = delta_theta / dt
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        # Update for next iteration
        self.last_left_enc = msg.mot_1_enc_val
        self.last_right_enc = msg.mot_2_enc_val
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
