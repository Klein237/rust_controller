#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose as TurtlePose
from geometry_msgs.msg import Pose2D

class TurtlePoseBridge(Node):
    def __init__(self):
        super().__init__('turtle_pose_bridge')
        self.subscriber = self.create_subscription(
            TurtlePose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.publisher = self.create_publisher(
            Pose2D,
            '/robot_pose',
            10
        )

    def pose_callback(self, msg):
        pose = Pose2D()
        pose.x = float(msg.x)
        pose.y = float(msg.y)
        pose.theta = float(msg.theta)
        self.publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
