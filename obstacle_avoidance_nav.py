#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations
import time


class ObstacleAvoidanceNavigator(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_navigator')
        self.navigator = BasicNavigator()

        self.get_logger().info("Waiting for Nav2 stack to become active...")
        self.navigator.waitUntilNav2Active()

        self.get_logger().info("Starting obstacle-avoiding navigation...")

        # List of goals to navigate
        self.goals = [
            self.create_pose(1.5, 0.5, 0.0),
            self.create_pose(0.5, 2.5, 1.57),
            self.create_pose(2.5, 2.5, 3.14),
        ]

        self.navigate_goals()

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def navigate_goals(self):
        for i, goal in enumerate(self.goals):
            self.get_logger().info(f"Navigating to goal {i + 1}")
            self.navigator.goToPose(goal)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
                time.sleep(1.0)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Goal {i + 1} reached successfully.")
            elif result == TaskResult.CANCELED:
                self.get_logger().warn("Goal was canceled.")
            elif result == TaskResult.FAILED:
                self.get_logger().error("Goal failed. Obstacle may have blocked the path.")

        self.get_logger().info("All goals completed or attempted.")
        self.navigator.lifecycleShutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNavigator()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
