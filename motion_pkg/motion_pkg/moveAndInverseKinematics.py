#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from motion_pkg.port_handler import PortHandler
from motion_pkg.packet_handler import PacketHandler
from motion_pkg.robotics_def import *
from robot_interfaces import *
from robot_interfaces.msg import OneMove
from robot_interfaces.srv import GetPosition
import math
import time

import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Control table address
ADDR_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 30
ADDR_PRESENT_POSITION   = 36
ADDR_MOVING_SPEED       = 32
ADDR_CW_LIMIT = 6
ADDR_CCW_LIMIT = 8

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                  # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

class pyNode(Node):
    def __init__(self):
        super().__init__('moveAndInverseKinematics')
        self.subscriber_ = self.create_subscription(OneMove, 'which_position', self.set_goal_pos_callback, 10)
        self.previous_message = None
        self.get_logger().info("Robot")

    def set_goal_pos_callback(self, data):
        def c(val):
            return val * 1023 / 300
        
        def anglesProduce(position: str, yf):
            R = 3
            d = 0.8                         # Distance between robotic arm and chess board
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
                degree = math.degrees(math.atan((side*(hor - 5) + (side/2)) / (R + d + side*(ver - 1) + (side/2))))
                print("degree= ", degree)
                return 150 - degree

            xf = findXf(position)
            # yf = 0

            l1 = 15.75
            l2 = 13.25
            l3 = 17.40
            
            k = (xf ** 2) + (yf ** 2) + (l1 ** 2) - (l2 ** 2) + (2 * yf * l3) + (l3 ** 2)
            n = math.sqrt(((2 * xf * l1) ** 2) + ((2 * l1 * yf + 2 * l1 * l3) ** 2))

            alphr = math.atan((2 * yf * l1 + 2 * l1 * l3) / (2 * xf * l1))
            alphd = math.degrees(alphr)
            thetas = []

            print("k: ", k)
            print("n: ", n)

            theta11 = alphd + math.degrees(math.acos(k / n))
            theta12 = alphd - math.degrees(math.acos(k / n))

            theta21 = - theta11 + math.degrees(math.acos(((xf - l1 * math.cos(math.radians(theta11))) / l2)))
            theta22 = - theta12 + math.degrees(math.acos(((xf - l1 * math.cos(math.radians(theta12))) / l2)))

            theta31 = -90 - theta11 - theta21
            theta32 = -90 - theta12 - theta22

            thetas.extend([theta11, theta12, theta21, theta22, theta31, theta32])

            for i in range(len(thetas)):
                if thetas[i] < 0:
                    thetas[i] = thetas[i] + 360
                elif thetas[i] > 360:
                    thetas[i] = thetas[i] - 360

            for i in range(2, len(thetas)):
                thetas[i] = thetas[i] - 120

            thetas[0] = thetas[0] + 60 + 20       # motor 2
            thetas[2] = 300 - thetas[2]      # motor 3
            thetas[4] = thetas[4] + 60            # motor 4

            for i in range(len(thetas)):
                thetas[i] = thetas[i] * 1023 / 300;

            return [c(findYawRad(position)), thetas[0], thetas[2], thetas[4]]

        def moveToBefore(loc: str, speed=81):
            chessPosition = anglesProduce(loc, -3)
            chessPosition.append(c(50))
            for servo_id, position in zip(servo_ids, chessPosition):
                print("Set Goal Position of ID before %s = %s, %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

        def moveToAfter(loc: str, speed=81):
            chessPosition = anglesProduce(loc, -9)
            chessPosition.append(c(0))
            for servo_id, position in zip(servo_ids, chessPosition):
                print("Set Goal Position of ID after %s = %s, %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

        servo_ids = [1, 2, 3, 4, 5]

        # servos_default = [512, 512, 512, 512, openGripper]``
        # for servo_id, position in zip(servo_ids, servos_default):
        #     print("Set Goal Position of ID %s = %s" % (servo_id, int(position)))
        #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
        
        # servos_test = [512, c(163), c(158), c(151), 0]
        # servos_goal = anglesProduce(data.location, 5)
        # servos_goal.append(closeGripper);
        # for servo_id, position in zip(servo_ids, servos_test):
        #     print("Set Goal Position of ID %s = %s" % (servo_id, int(position)))
        #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

        defaultPosition = [512,c(185),c(178),c(138),c(0)]

        desired_speed = 81

        for servo_id, position in zip(servo_ids, defaultPosition):
            print("Set Goal Position of ID default %s = %s, %s" % (servo_id, int(position), position*300/1023))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, 81)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

        time.sleep(5)
        moveToBefore("a5")
        time.sleep(5)
        moveToAfter("a5")
        time.sleep(5)
        moveToBefore("a5")

        # time.sleep(2)

        # for servo_id, position in zip(servo_ids, chessPosition['a8'][5:]):
        #     print("Set Goal Position of ID %s = %s" % (servo_id, int(position)))
        #     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, 81)
        #     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
        #     # for i in range(int(position)):
        #     #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(i))
        #     #     time.sleep(0.5)

        # time.sleep(5)

        # for servo_id, position in zip(servo_ids, chessPosition['a8'][:5]):
        #     print("Set Goal Position of ID %s = %s" % (servo_id, int(position)))
        #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
        #     # dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, desired_speed)
        #     # for i in range(int(position)):
        #     #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(i))
        #     #     time.sleep(0.5)

        # time.sleep(5)

        # servos_default = [512, 512, 512, 512, openGripper]
        # for servo_id, position in zip(servo_ids, servos_default):
        #     print("Set Goal Position of ID %s = %s" % (servo_id, int(position)))
        #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

def read_write_py_node(args=None):
    rclpy.init(args=args)
    node = pyNode()
    rclpy.spin(node)
    rclpy.shutdown()

def main(args=None):
    print("test")
    # Open port
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL has been successfully connected")

    print("Ready to get & set Position.")

    read_write_py_node(args)

if __name__ == '__main__':
    main()