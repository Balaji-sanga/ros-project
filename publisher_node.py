import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self, n):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, '/chat', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.n = n

    def timer_callback(self):
        if self.count < self.n:
            msg = String()
            msg.data = 'hello world'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.count += 1
        else:
            self.get_logger().info(f'Published {self.n} messages. Shutting down...')
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    n = 10  # Change this value for your required count
    node = HelloWorldPublisher(n)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
