#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from motion_pkg.port_handler import PortHandler
from motion_pkg.packet_handler import PacketHandler
from motion_pkg.robotics_def import *
from robot_interfaces import *
from robot_interfaces.msg import SetPosition
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
        super().__init__('speedster')
        self.subscriber_ = self.create_subscription(SetPosition, 'set_position', self.set_goal_pos_callback, 10)
        # self.service_ = self.create_service(GetPosition, 'get_position', self.get_present_pos)
        self.get_logger().info("Hello World")

    def set_goal_pos_callback(self, data):
        print("Set Goal Position of ID %s = %s" % (data.id, data.position))
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, data.id, ADDR_MOVING_SPEED, 64)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)

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