import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
import time

class LetterDrawer(Node):
    def __init__(self):
        super().__init__('letter_drawer')

        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for teleport service...")
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for pen service...")

        self.draw_all_letters()

    def teleport(self, x, y, theta=0.0):
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        self.teleport_client.call_async(req)
        time.sleep(0.2)

    def set_pen(self, on=True, r=255, g=0, b=0):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = 2
        req.off = not on
        self.pen_client.call_async(req)
        time.sleep(0.1)

    def draw_line(self, x1, y1, x2, y2):
        self.set_pen(False)
        self.teleport(x1, y1)
        self.set_pen(True)
        self.teleport(x2, y2)

    def draw_A(self, offset_x=1.0):
        self.get_logger().info("Drawing A")
        self.draw_line(offset_x, 1.0, offset_x + 0.5, 3.0)
        self.draw_line(offset_x + 0.5, 3.0, offset_x + 1.0, 1.0)
        self.draw_line(offset_x + 0.25, 2.0, offset_x + 0.75, 2.0)

    def draw_B(self, offset_x=3.0):
        self.get_logger().info("Drawing B")
        self.draw_line(offset_x, 1.0, offset_x, 3.0)
        self.draw_line(offset_x, 3.0, offset_x + 0.7, 2.6)
        self.draw_line(offset_x + 0.7, 2.6, offset_x, 2.0)
        self.draw_line(offset_x, 2.0, offset_x + 0.7, 1.6)
        self.draw_line(offset_x + 0.7, 1.6, offset_x, 1.0)

    def draw_C(self, offset_x=5.0):
        self.get_logger().info("Drawing C")
        self.draw_line(offset_x + 0.8, 3.0, offset_x, 3.0)
        self.draw_line(offset_x, 3.0, offset_x, 1.0)
        self.draw_line(offset_x, 1.0, offset_x + 0.8, 1.0)

    def draw_all_letters(self):
        self.draw_A(offset_x=1.0)
        self.draw_B(offset_x=3.0)
        self.draw_C(offset_x=5.0)
        self.get_logger().info("Finished drawing all letters.")

def main(args=None):
    rclpy.init(args=args)
    node = LetterDrawer()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
