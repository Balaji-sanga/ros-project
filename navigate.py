#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time


class TurtleBotNavigator(Node):
    def __init__(self):
        super().__init__('turtlebot3_nav_node')
        self.navigator = BasicNavigator()

        # Wait for navigation to activate
        self.get_logger().info("Waiting for Nav2 to be ready...")
        self.navigator.waitUntilNav2Active()

        # Define 3 hardcoded goal positions
        self.goals = [
            self.create_pose(1.0, 1.0, 0.0),
            self.create_pose(2.5, 0.5, 1.57),  # 90 degrees
            self.create_pose(0.0, 2.5, 3.14),  # 180 degrees
        ]

        self.navigate_to_goals()

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Convert yaw to quaternion
        import tf_transformations
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def navigate_to_goals(self):
        for i, goal in enumerate(self.goals):
            self.get_logger().info(f"Navigating to goal {i+1}")
            self.navigator.goToPose(goal)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f'Feedback: {feedback.distance_remaining:.2f} m remaining')
                time.sleep(1.0)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Goal {i+1} reached!")
            else:
                self.get_logger().warn(f"Failed to reach goal {i+1}")

        self.get_logger().info("Navigation to all goals complete.")
        self.navigator.lifecycleShutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotNavigator()
    node.destroy_node()
    rclpy.shutdown()
