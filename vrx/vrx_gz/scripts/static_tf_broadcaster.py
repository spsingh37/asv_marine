#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')

        # Initialize the static transform broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Define the static transform for the new frame at the Gazebo origin
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'  # 'world' as the parent frame (Gazebo's world)
        t.child_frame_id = 'odom'  # New frame to define at the origin

        # Set translation to zero (origin of the Gazebo world)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set rotation to zero (aligned with the world frame, no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the static transform
        self.static_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()