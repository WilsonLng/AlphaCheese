import rclpy
from rclpy.node import Node

class array_subscriber_node(Node):
    def __init__(self):
        super().__init__("array_subscribe")
        self.subscriber_ = self.create_subscription(str, "array_re", self.listen, 10)
        self.subscriptions

    def listen(self, msg):
        received_array = msg
        self.get_logger().info(f"Received array: {received_array}")


def main(args = None):
    rclpy.init(args=args)
    array_subsriber = array_subscriber_node()
    rclpy.spin(array_subsriber)
    array_subsriber.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()