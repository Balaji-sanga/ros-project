import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MultiTopicSubscriber(Node):
    def __init__(self):
        super().__init__('multi_topic_subscriber')
        self.sub1 = self.create_subscription(Int32, 'chat1', self.callback1, 10)
        self.sub2 = self.create_subscription(Int32, 'chat2', self.callback2, 10)
        self.sub3 = self.create_subscription(Int32, 'chat3', self.callback3, 10)

        self.value1 = 0
        self.value2 = 0
        self.value3 = 0

        self.timer = self.create_timer(2.0, self.timer_callback)

    def callback1(self, msg):
        self.value1 = msg.data

    def callback2(self, msg):
        self.value2 = msg.data

    def callback3(self, msg):
        self.value3 = msg.data

    def timer_callback(self):
        total = self.value1 + self.value2 + self.value3
        self.get_logger().info(f'Sum of values: {self.value1} + {self.value2} + {self.value3} = {total}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

