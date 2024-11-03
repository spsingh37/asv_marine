#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry  # Assuming you are getting odometry for the base_link

class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')

        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the odometry (or pose) topic to get the base_link's position
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/gtodom',  # Topic where odometry data is published, you can change it to match your setup
            self.handle_odometry,
            10
        )

    def handle_odometry(self, msg):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the parent frame (gazebo_origin) and child frame (base_link)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame
        t.child_frame_id = 'wamv/wamv/base_link'  # Child frame

        # Set the translation (position of base_link relative to gazebo_origin)
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Set the rotation (orientation of base_link relative to gazebo_origin)
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
