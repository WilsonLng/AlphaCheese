#!/usr/bin/env python

#*******************************************************************************
# Copyright 2021 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************

#*******************************************************************************
# This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
# For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
# To test this example, please follow the commands below.
#
# Open terminal #1
# $ roscore
#
# Open terminal #2
# $ rosrun dynamixel_sdk_examples read_write_node.py
#
# Open terminal #3 (run one of below commands at a time)
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
# $ rosservice call /get_position "id: 1"
#
# Author: Will Son
#******************************************************************************/

# Modified by Wilson
# Changed from ROS to ROS2 for compatibility

import os
import rclpy # Modified to rclpy from rospy
from rclpy.node import Node # Importing node from rclpy, previously not done
from motion_pkg.port_handler import PortHandler
from motion_pkg.packet_handler import PacketHandler
from motion_pkg.robotics_def import *
from robot_interfaces import *
from robot_interfaces.msg import SetPosition
from robot_interfaces.srv import GetPosition
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
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
ADDR_MOVING_SPEED       = 32
ADDR_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                  # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Created class for ease of use, previously no class
class pyNode(Node):
    def __init__(self):
        super().__init__('read_write_py_node')
        self.subscriber_ = self.create_subscription(SetPosition, 'set_position', self.set_goal_pos_callback, 10)
        self.service_ = self.create_service(GetPosition, 'get_position', self.get_present_pos)
        self.get_logger().info("Hello World")

    def set_goal_pos_callback(self, data):
        def c(val):
            return val * 1023 / 300
        
        print("Set Goal Position of ID %s = %s" % (data.id, data.position))
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVING_SPEED, 59)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(133)))

        time.sleep(3)

        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVING_SPEED, 59)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(114)))

    # Added third argument as callback doesn't work with just two
    def get_present_pos(self, req, res):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
        print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
        # remove return as it is not needed
        res.position = int(dxl_present_position)

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