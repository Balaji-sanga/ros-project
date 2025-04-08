import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import math
import random
import time

class RandomCircleDrawer(Node):
    def __init__(self):
        super().__init__('random_circle_drawer')

        # Create service clients
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Wait for services
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pen service...')

        self.draw_random_circle()

    def draw_random_circle(self):
        # Random center and radius
        radius = random.uniform(1.0, 4.0)
        center_x = random.uniform(radius + 1.0, 10.0 - radius)
        center_y = random.uniform(radius + 1.0, 10.0 - radius)

        self.get_logger().info(f'Drawing circle at ({center_x:.2f}, {center_y:.2f}) with radius {radius:.2f}')

        # Move turtle to starting point of the circle (angle = 0)
        start_x = center_x + radius
        start_y = center_y
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = float(start_x)
        teleport_req.y = float(start_y)
        teleport_req.theta = float(math.pi / 2)  # Face upward
        self.teleport_client.call_async(teleport_req)

        # Enable pen
        pen_req = SetPen.Request()
        pen_req.r = random.randint(0, 255)
        pen_req.g = random.randint(0, 255)
        pen_req.b = random.randint(0, 255)
        pen_req.width = 2
        pen_req.off = False
        self.pen_client.call_async(pen_req)

        time.sleep(1.0)  # Let turtle settle before drawing

        # Draw the circle using velocity commands
        twist = Twist()
        twist.linear.x = 2.0  # Speed = radius * angular speed
        twist.angular.z = 2.0 / radius  # Full circle: theta = 2π

        duration = (2 * math.pi) / twist.angular.z  # Time to complete one circle
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration

        self.get_logger().info(f'Drawing for {duration:.2f} seconds...')

        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)

        # Stop the turtle
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Finished drawing the circle.")

def main(args=None):
    rclpy.init(args=args)
    node = RandomCircleDrawer()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

