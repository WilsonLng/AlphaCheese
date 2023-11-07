import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import random


class array_publisher_node(Node):
    def __init__(self):
        super().__init__("array_publish")
        self.array_publisher = self.create_publisher(str, "array", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.publish_array)

    
    def publish_array(self):
        array = generate_random_array()
        msg = String()
        msg.data = array
        self.array_publisher.publish(msg)


def generate_random_array():
    array = [['-' for _ in range(8)] for _ in range(8)]

    for _ in range(16):
        while True:
            x, y = random.randint(0, 7), random.randint(0, 7)
            if array[x][y] == '-':
                array[x][y] = 'b'
                break

    for _ in range(16):
        while True:
            x, y = random.randint(0, 7), random.randint(0, 7)
            if array[x][y] == '-':
                array[x][y] = 'w'
                break

    print(array)

def main(args=None):
    rclpy.init(args=args)
    node = array_publisher_node()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()