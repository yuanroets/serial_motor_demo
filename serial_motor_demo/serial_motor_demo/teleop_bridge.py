#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial_motor_demo_msgs.msg import MotorCommand
import math

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        
        # Subscribe to cmd_vel from teleop
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to motor_command (your driver's topic)
        self.motor_publisher = self.create_publisher(
            MotorCommand,
            'motor_command',
            10
        )
        
        # Robot parameters (adjust these for your robot)
        self.declare_parameter('wheel_separation', 0.297)  # Distance between wheels (m)
        self.declare_parameter('wheel_radius', 0.033)      # Wheel radius (m)
        self.declare_parameter('max_linear_speed', 1.0)    # Max linear speed (m/s)
        self.declare_parameter('max_angular_speed', 2.0)   # Max angular speed (rad/s)
        
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        self.get_logger().info(f'Teleop bridge started - wheel_sep: {self.wheel_separation}m, wheel_radius: {self.wheel_radius}m')
        
    def cmd_vel_callback(self, msg):
        """Convert Twist message to differential drive motor commands"""
        
        # Extract linear and angular velocities
        linear_vel = msg.linear.x   # m/s
        angular_vel = msg.angular.z # rad/s
        
        # Limit velocities
        linear_vel = max(min(linear_vel, self.max_linear_speed), -self.max_linear_speed)
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)
        
        # Convert to differential drive wheel velocities
        # For correct turning direction, negate angular velocity
        v_left = linear_vel - (-angular_vel * self.wheel_separation / 2.0)
        v_right = linear_vel + (-angular_vel * self.wheel_separation / 2.0)
        
        # Convert wheel velocities to rad/s
        wheel_left_rad_s = v_left / self.wheel_radius
        wheel_right_rad_s = v_right / self.wheel_radius
        
        # Create motor command
        motor_cmd = MotorCommand()
        motor_cmd.is_pwm = False  # Use velocity control, not PWM
        motor_cmd.mot_1_req_rad_sec = float(wheel_right_rad_s)  # Adjust motor mapping as needed
        motor_cmd.mot_2_req_rad_sec = float(wheel_left_rad_s)   # Adjust motor mapping as needed
        
        # Log the conversion
        self.get_logger().debug(
            f'Teleop: lin={linear_vel:.2f}, ang={angular_vel:.2f} -> '
            f'Motors: M1={motor_cmd.mot_1_req_rad_sec:.2f}, M2={motor_cmd.mot_2_req_rad_sec:.2f}'
        )
        
        # Publish motor command
        self.motor_publisher.publish(motor_cmd)

def main(args=None):
    rclpy.init(args=args)
    
    teleop_bridge = TeleopBridge()
    
    try:
        rclpy.spin(teleop_bridge)
    except KeyboardInterrupt:
        pass
    
    teleop_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
