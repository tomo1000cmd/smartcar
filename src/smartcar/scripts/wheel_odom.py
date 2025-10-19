#!/usr/bin/env python3
"""
Wheel Odometry Publisher for Smartcar (Assignment Requirements)
Single input single output system: input=/smart_car/vehicle_status, output=/smart_car/wheel/odom
Implements the assignment's specific kinematic model and covariance matrix.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from smartcar_msgs.msg import Status
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        
        # Get use_sim_time parameter (automatically declared by ROS 2)
        try:
            use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        except:
            use_sim_time = True  # Default to simulation time
        
        # Robot parameters from assignment
        self.wheel_diameter = 0.064  # D = 0.064 meters (assignment specification)
        self.wheelbase = 0.257       # Distance between front and rear axles
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.initialized = False
        
        # Covariance values from assignment
        self.position_variance = 1.0
        self.orientation_variance = 0.5
        self.unused_variance = 100000.0
        self.linear_velocity_variance = 0.1
        self.angular_velocity_variance = 0.1
        
        # Subscribe to vehicle status (single input as per assignment)
        self.status_sub = self.create_subscription(
            Status,
            '/smartcar/vehicle_status',
            self.status_callback,
            10
        )
        
        # Publish odometry (single output as per assignment)
        self.odom_pub = self.create_publisher(Odometry, '/smart_car/wheel/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Wheel Odometry Node started')
    
    def status_callback(self, msg):
        """
        Process vehicle status and calculate odometry using assignment specifications.
        Input: /smart_car/vehicle_status with engine_speed_rpm, steering_angle_rad
        Output: /smart_car/wheel/odom
        """
        current_time = self.get_clock().now()
        
        if not self.initialized:
            self.last_time = current_time
            self.initialized = True
            return
        
        # Calculate time step using system clock (as per assignment)
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        
        # Assignment formula: v = RPM * π * D / 60
        # Where: v = Linear Velocity, RPM = engine_speed_rpm, D = wheel_diameter
        rpm = msg.engine_speed_rpm
        v = (rpm * math.pi * self.wheel_diameter) / 60.0  # Linear velocity in robot frame
        
        # Assignment kinematic model for angular velocity:
        # ω = (v / L) * tan(δ)
        # Where: v = Linear Velocity, L = wheelbase, δ = steering_angle
        steering_angle = msg.steering_angle_rad
        if abs(steering_angle) < 1e-6:
            omega = 0.0  # Straight line motion
        else:
            omega = (v / self.wheelbase) * math.tan(steering_angle)
        
        # Assignment integration model for position:
        # x(k+1) = x(k) + v * cos(θ) * dt
        # y(k+1) = y(k) + v * sin(θ) * dt  
        # θ(k+1) = θ(k) + ω * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry with assignment covariance values
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from yaw
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocities
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        
        # Assignment covariance matrix (diagonal components)
        # Position covariance (x, y, z)
        odom.pose.covariance[0] = self.position_variance    # x-x
        odom.pose.covariance[7] = self.position_variance    # y-y  
        odom.pose.covariance[14] = self.unused_variance     # z-z
        
        # Orientation covariance (roll, pitch, yaw)
        odom.pose.covariance[21] = self.unused_variance     # roll-roll
        odom.pose.covariance[28] = self.unused_variance     # pitch-pitch
        odom.pose.covariance[35] = self.orientation_variance # yaw-yaw
        
        # Velocity covariance (linear x, y, z)
        odom.twist.covariance[0] = self.linear_velocity_variance   # linear x-x
        odom.twist.covariance[7] = self.unused_variance           # linear y-y
        odom.twist.covariance[14] = self.unused_variance          # linear z-z
        
        # Angular velocity covariance (roll, pitch, yaw rates)
        odom.twist.covariance[21] = self.unused_variance          # angular x-x
        odom.twist.covariance[28] = self.unused_variance          # angular y-y  
        odom.twist.covariance[35] = self.angular_velocity_variance # angular z-z
        
        self.odom_pub.publish(odom)
        
        # Publish TF transform from odom to base_footprint
        # Disabled: EKF handles TF publishing to avoid conflicts
        # t = TransformStamped()
        # t.header.stamp = current_time.to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_footprint'
        # 
        # # Translation
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # 
        # # Rotation (quaternion from yaw)
        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = math.sin(self.theta / 2.0)
        # t.transform.rotation.w = math.cos(self.theta / 2.0)
        # 
        # self.tf_broadcaster.sendTransform(t)
        
        # Update last time
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
