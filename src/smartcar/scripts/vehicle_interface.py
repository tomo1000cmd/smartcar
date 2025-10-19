#!/usr/bin/env python3
"""
Vehicle Interface Node for Smartcar (Ackermann Drive)
Subscribes to Nav2 cmd_vel and converts to Ackermann drive commands.
Publishes to /cmd_vel which is consumed by the car_gazebo_plugin.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class VehicleInterface(Node):
    def __init__(self):
        super().__init__('vehicle_interface')
        
        # Parameters
        self.declare_parameter('wheelbase', 0.257)  # Distance between front and rear axles
        self.declare_parameter('track_width', 0.17)  # Distance between left and right wheels
        self.declare_parameter('max_steering_angle', 0.7854)  # 45 degrees in radians
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        
        # Subscribe to Nav2 cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to Gazebo cmd_vel (car_gazebo_plugin Ackermann)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Vehicle Interface Node started')
        self.get_logger().info(f'Wheelbase: {self.wheelbase}m, Track width: {self.track_width}m')
    
    def cmd_vel_callback(self, msg):
        """
        Process cmd_vel from Nav2 and ensure commands respect Ackermann constraints.
        The car_gazebo_plugin will convert (v, omega) to steering angle and wheel velocities.
        """
        output_msg = Twist()
        
        # Pass through linear velocity
        output_msg.linear.x = msg.linear.x
        output_msg.linear.y = msg.linear.y
        output_msg.linear.z = msg.linear.z
        
        # Adjust angular velocity to respect Ackermann steering limits
        if abs(msg.linear.x) > 0.001 and abs(msg.angular.z) > 0.001:
            # Calculate turning radius from angular velocity
            # omega = v / R => R = v / omega
            turning_radius = msg.linear.x / msg.angular.z
            
            # For Ackermann steering, the minimum turning radius is:
            # R_min = wheelbase / tan(max_steering_angle)
            min_radius = self.wheelbase / math.tan(self.max_steering_angle)
            
            # Limit the turning radius to respect maximum steering angle
            if abs(turning_radius) < min_radius:
                # Adjust angular velocity to respect steering limits
                turning_radius = min_radius if turning_radius > 0 else -min_radius
                output_msg.angular.z = msg.linear.x / turning_radius
            else:
                output_msg.angular.z = msg.angular.z
        else:
            output_msg.angular.z = msg.angular.z
        
        # Publish adjusted command
        self.cmd_vel_pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
