#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')

        # Subscribe to the /tf topic, which publishes dynamic transforms
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',  # Topic where transforms are published
            self.handle_tf_message,
            10
        )

        # Create a TransformBroadcaster for dynamic transforms
        self.br = TransformBroadcaster(self)

    def handle_tf_message(self, msg):
        # Iterate through each transform in the TFMessage
        for transform in msg.transforms:
            # Check if the transform is between 'world' and 'wamv/wamv/base_link'
            if transform.header.frame_id == 'world' and transform.child_frame_id == 'wamv/wamv/base_link':
                # Create a TransformStamped message
                t = TransformStamped()

                # Populate TransformStamped with the robot pose relative to world frame
                t.header.stamp = transform.header.stamp
                t.header.frame_id = 'world'  # Fixed frame
                t.child_frame_id = 'wamv/wamv/base_link'  # Robot's base frame

                # Set translation and rotation from the original message
                t.transform.translation = transform.transform.translation
                t.transform.rotation = transform.transform.rotation

                # Broadcast the transform
                self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    dynamic_tf_broadcaster = DynamicTFBroadcaster()
    rclpy.spin(dynamic_tf_broadcaster)
    dynamic_tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from tf2_ros import TransformBroadcaster
# from tf2_msgs.msg import TFMessage
# from geometry_msgs.msg import TransformStamped

# class DynamicTFBroadcaster(Node):
#     def __init__(self):
#         super().__init__('dynamic_tf_broadcaster')
        
#         # Subscribe to the TFMessage topic
#         self.pose_sub = self.create_subscription(
#             TFMessage,
#             '/wamv/pose',  # Topic where Gazebo publishes the robot's pose
#             self.handle_tf_message,
#             10
#         )

#         # Create a TransformBroadcaster for dynamic transforms
#         self.br = TransformBroadcaster(self)

#     def handle_tf_message(self, msg):
#         # Iterate through each transform in the TFMessage
#         for transform in msg.transforms:
#             # Create a TransformStamped message
#             t = TransformStamped()
#             # Populate TransformStamped with robot pose relative to world frame
#             t.header.stamp = transform.header.stamp
#             t.header.frame_id = transform.header.frame_id  # Frame of the transform
#             t.child_frame_id = transform.child_frame_id  # Child frame of the transform

#             # Set translation and rotation
#             t.transform.translation = transform.transform.translation
#             t.transform.rotation = transform.transform.rotation

#             # Broadcast the transform
#             self.br.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     dynamic_tf_broadcaster = DynamicTFBroadcaster()
#     rclpy.spin(dynamic_tf_broadcaster)
#     dynamic_tf_broadcaster.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped

# class DynamicTFBroadcaster(Node):
#     def __init__(self):
#         super().__init__('dynamic_tf_broadcaster')

#         # Subscribe to the Gazebo pose topic to get absolute robot pose
#         self.pose_sub = self.create_subscription(
#                 PoseStamped,
#                 '/wamv/pose',  # Topic where Gazebo publishes the robot's pose (adjust this if needed)
#                 self.handle_pose,
#                 10)

#         # Create a TransformBroadcaster for dynamic transforms
#         self.br = TransformBroadcaster(self)

#         self.timer = self.create_timer(0.1, self.publish_transform)

#     def handle_pose(self, msg):
#         # Create a TransformStamped message
#         t = TransformStamped()
#         # Populate TransformStamped with robot pose relative to world frame
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = 'world'  # Fixed frame
#         t.child_frame_id = 'wamv/wamv/base_link'  # Robot's base frame

#         # Set translation (absolute position)
#         t.transform.translation.x = msg.pose.position.x
#         t.transform.translation.y = msg.pose.position.y
#         t.transform.translation.z = msg.pose.position.z

#         # Set rotation (absolute orientation)
#         t.transform.rotation = msg.pose.orientation

#         # Broadcast the transform
#         self.br.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DynamicTFBroadcaster()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
