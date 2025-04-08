import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
from geometry_msgs.msg import Twist
import time

class ZigzagCleaner(Node):
    def __init__(self):
        super().__init__('zigzag_cleaner')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for teleport service...")

        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_pen service...")

        self.zigzag()

    def zigzag(self):
        # Teleport to starting point
        self.teleport(0.5, 0.5, 0.0)
        self.set_pen(True)
        self.get_logger().info("Starting Zigzag cleaning...")

        row_height = 1.0
        y = 0.5
        forward = True

        while y <= 10.5:
            if forward:
                self.move_to(10.5, y)
            else:
                self.move_to(0.5, y)
            y += row_height
            if y <= 10.5:
                self.move_to(self.get_last_x(), y)
            forward = not forward

        self.get_logger().info("Finished Zigzag cleaning.")
        self.stop()

    def teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        self.teleport_client.call_async(req)
        time.sleep(0.2)

    def set_pen(self, on):
        req = SetPen.Request()
        req.r = 0
        req.g = 0
        req.b = 255
        req.width = 2
        req.off = not on
        self.pen_client.call_async(req)
        time.sleep(0.1)

    def move_to(self, x_target, y_target):
        msg = Twist()
        current_x = self.get_last_x()
        msg.linear.x = 2.0 if x_target > current_x else -2.0
        distance = abs(x_target - current_x)

        travel_time = distance / abs(msg.linear.x)
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        end_time = start_time + travel_time

        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.vel_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.vel_publisher.publish(Twist())
        time.sleep(0.2)

    def get_last_x(self):
        # Since we don't subscribe to Pose, we just alternate based on last target
        # In real robots, you'd track the pose
        return 0.5 if self.last_x == 10.5 else 10.5 if hasattr(self, 'last_x') else 0.5

    def stop(self):
        self.vel_publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ZigzagCleaner()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

