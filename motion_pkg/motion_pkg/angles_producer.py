import rclpy
from rclpy.node import Node
from robot_interfaces.msg import PiecePosition 
from robot_interfaces.msg import DecimalPosition
import math

class angleNode(Node):
    def __init__(self):
        super().__init__(angleNode)
        self.publisher = self.create_publisher(DecimalPosition, 'decimal_position', 10)
        self.subscription = self.create_subscription(PiecePosition, 'piece_position', self.callback, 10)
        self.get_logger().info('Node is ready')

    def callback(self, msg):
        output_msg = DecimalPosition()

        R = 3
        d = 1.5                         # Distance between robotic arm and chess board
        side = 2.75

        num = 1
        hordict = {}

        for i in "abcdefgh":
            hordict[i] = num
            num +=1

        def findXf(pos):
            ver = int(pos[1])
            hor = hordict[pos[0]]
            return (R + ((R + d + side*(ver - 1) + (side/2))**2 + (side*(hor - 5) + (side/2))**2)**(1/2))

        def findYawRad(pos):
            ver = int(pos[1])
            hor = hordict[pos[0]]
            return (math.atan((side*(hor - 5) + (side/2)) / (R + d + side*(ver - 1) + (side/2))))*(180/math.pi)+150

        xf = findXf(msg.data)
        yf = 0

        l1 = 15.75
        l2 = 13.25
        l3 = 17.40
        
        k = xf ** 2 + yf ** 2 + l1 ** 2 - l2 ** 2 + 2 * yf * l3 + l3 ** 2
        n = math.sqrt((2 * xf * l1) ** 2 + (2 * yf * l1 + 2 * l1 * l3) ** 2)

        alphr = math.atan((2 * yf * l1 + 2 * l1 * l3) / (2 * xf * l1))
        alphd = math.degrees(alphr)

        thetas = []

        theta11 = alphd + math.degrees(math.acos(k / n))
        theta12 = alphd - math.degrees(math.acos(k / n))

        theta21 = - theta11 + math.degrees(math.acos(((xf - l1 * math.cos(math.radians(theta11))) / l2)))
        theta22 = - theta12 + math.degrees(math.acos(((xf - l1 * math.cos(math.radians(theta12))) / l2)))

        theta31 = 270 - theta11 - theta21
        theta32 = 270 - theta12 - theta22

        thetas.extend([theta11, theta12, theta21, theta22, theta31, theta32])

        for i in range(len(thetas)):
            if thetas[i] < 0:
                thetas[i] = thetas[i] + 360
            elif thetas[i] > 360:
                thetas[i] = thetas[i] - 360

        for i in range(2, len(thetas)):
            thetas[i] = thetas[i] - 120

        thetas[0] = thetas[0] + 60
        thetas[1] = thetas[1] + 60
        thetas[2] = 300 - thetas[2]

        output_msg.angle1 = findYawRad(msg.data)
        output_msg.angle2 = thetas[0]
        output_msg.angle3 = thetas[2]
        output_msg.angle4 = thetas[4]

        # Populate the output message
        self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = angleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()