#!/usr/bin/env python3
"""
Joint State Publisher for Smartcar in Gazebo
Publishes static joint states for fixed joints if needed.
Note: Gazebo typically publishes joint states automatically.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Publisher
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos)
        
        # Timer for publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Joint names (fixed joints that might not be published by Gazebo)
        self.joint_names = [
            'base_footprint_joint',
            'chassis_joint',
            'imu_joint',
            'lidar_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint',
            'front_left_wheel_steer_joint',
            'front_right_wheel_steer_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint'
        ]
        
        self.get_logger().info('Joint State Publisher Node started')
        self.get_logger().info('Note: Gazebo publishes most joint states automatically')
    
    def publish_joint_states(self):
        """Publish joint states for fixed joints"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
