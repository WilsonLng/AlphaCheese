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
        super().__init__('moveAndTake')
        self.subscriber_ = self.create_subscription(OneMove, 'which_position', self.set_goal_pos_callback, 10)
        self.previous_message = None
        self.get_logger().info("Robot")

    def set_goal_pos_callback(self, data):
        def c(val):
            return val * 1023 / 300
        
        chessPosition = {
            'a1': [],
            'a2': [],
            'a3': [],
            'a4': [],
            'a5': [],
            'a6': [],
            'a7': [],
            'a8': [c(175),c(98),c(63),c(136),c(0),c(175),c(135),c(100),c(159),c(50)],
            'b1': [],
            'b2': [],
            'b3': [],
            'b4': [],
            'b5': [],
            'b6': [],
            'b7': [],
            'b8': [],
            'c1': [],
            'c2': [],
            'c3': [],
            'c4': [],
            'c5': [],
            'c6': [],
            'c7': [],
            'c8': [],
            'd1': [],
            'd2': [],
            'd3': [],
            'd4': [],
            'd5': [],
            'd6': [],
            'd7': [],
            'd8': [],
            'e1': [],
            'e2': [],
            'e3': [],
            'e4': [],
            'e5': [],
            'e6': [],
            'e7': [],
            'e8': [],
            'f1': [],
            'f2': [],
            'f3': [],
            'f4': [],
            'f5': [],
            'f6': [],
            'f7': [],
            'f8': [],
            'g1': [],
            'g2': [],
            'g3': [],
            'g4': [],
            'g5': [],
            'g6': [],
            'g7': [],
            'g8': [],
            'h1': [],
            'h2': [],
            'h3': [],
            'h4': [],
            'h5': [],
            'h6': [],
            'h7': [],
            'h8': [],
            'default': [512,c(185),c(178),c(138),c(0)],
            'out': []
        }

        def moveToBefore(loc: str, speed=81):
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MOVING_SPEED, speed)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, int(chessPosition[loc][5]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MOVING_SPEED, speed)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, int(c(220)))
            print("nsnsakdnkqkweknwqkneknwqkenkw")
            time.sleep(3)
            for servo_id, position in zip(servo_ids[2:], chessPosition[loc][7:]):
                print("Set Goal Position of ID %s = %s" % (servo_id, int(position)))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MOVING_SPEED, speed)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, int(chessPosition[loc][6]))

        def moveToAfter(loc: str, speed=81):
            for servo_id, position in zip(servo_ids, chessPosition[loc][:5]):
                print("Set Goal Position of ID %s = %s" % (servo_id, int(position)))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

        servo_ids = [1, 2, 3, 4, 5]

        # info = data.location.split()

        # if info[2] == 'x':
            #pass
            # '''
            #     Steps:
            #     1.default
            #     2.info[1][5:]
            #     3.info[1][:5]
            #     4.info[1][5:]
            #     5.out
            #     6.info[0][5:]
            #     7.info[0][:5]
            #     8.info[0][5:]
            #     9.info[1][5:]
            #     10.info[1][:5]
            #     11.info[1][5:]
            #     12.default
            # '''
        # elif info[2] == 'm':
            #pass
            # '''
            #     Steps:
            #     1.default
            #     2.info[0][5:]
            #     3.info[0][:5]
            #     4.info[0][5:]
            #     5.info[1][5:]
            #     6.info[1][:5]
            #     7.info[1][5:]
            #     8.default
            # '''
        # elif info[2] == '=':
            #pass
        # elif info[2] == '#':
            #pass
        # else:
            #pass


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

        desired_speed = 81

        for servo_id, position in zip(servo_ids, chessPosition['default']):
            print("Set Goal Position of ID %s = %s" % (servo_id, int(position)))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, 81)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

        time.sleep(5)
        moveToBefore("a8")
        time.sleep(5)
        moveToAfter("a8")
        time.sleep(5)
        moveToBefore("a8")

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