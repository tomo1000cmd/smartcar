#!/usr/bin/env python3
"""
Joystick to Twist Converter for Smartcar
Converts joystick commands to Twist messages for teleoperation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_twist')
        
        # Parameters for joystick mapping
        self.declare_parameter('linear_axis', 1)  # Left stick vertical
        self.declare_parameter('angular_axis', 3)  # Right stick horizontal
        self.declare_parameter('enable_button', 4)  # L1/LB button
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.enable_button = self.get_parameter('enable_button').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        
        # Subscribe to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Publish twist commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Joystick to Twist Node started')
        self.get_logger().info(f'Max speeds - Linear: {self.max_linear}m/s, Angular: {self.max_angular}rad/s')
    
    def joy_callback(self, msg):
        """Convert joystick input to Twist message"""
        twist = Twist()
        
        # Check if enable button is pressed (if button exists)
        if len(msg.buttons) > self.enable_button:
            if msg.buttons[self.enable_button] == 0:
                # Button not pressed, send zero velocity
                self.cmd_vel_pub.publish(twist)
                return
        
        # Get axis values (with bounds checking)
        if len(msg.axes) > self.linear_axis:
            linear_value = msg.axes[self.linear_axis]
            twist.linear.x = linear_value * self.max_linear
        
        if len(msg.axes) > self.angular_axis:
            angular_value = msg.axes[self.angular_axis]
            twist.angular.z = angular_value * self.max_angular
        
        # Publish twist command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
