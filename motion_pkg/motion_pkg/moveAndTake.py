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
import numpy as np

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

# class PIDController:
#     def __init__(self, Kp, Ki, Kd, goal):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.goal = goal
#         self.prev_error = 0
#         self.integral = 0

#     def update(self, current_position):
#         error = self.goal - current_position

#         P = self.Kp * error

#         self.integral += error
#         I = self.Ki * self.integral

#         D = self.Kd * (error - self.prev_error)

#         control_signal = P + I + D

#         self.prev_error = error

#         return control_signal

class pyNode(Node):
    def __init__(self):
        super().__init__('moveAndTake')
        self.subscriber_ = self.create_subscription(OneMove, 'which_position', self.set_goal_pos_callback, 10)
        self.service_ = self.create_service(GetPosition, 'get_position', self.get_present_pos)
        self.previous_message = None
        self.dxl_present_position = {}
        self.get_logger().info("Robot")

    def set_goal_pos_callback(self, data):
        def c(val):
            return val * 1023 / 300

        def d(val):
            return val * 300 / 1023

        def PID(Kp, Ki, Kd, MV_bar=0):
            # initialize stored data
            e_prev = 0
            t_prev = -100
            I = 0

            # initial control
            MV = MV_bar
            
            while True:
                # yield MV, wait for new t, PV, SP
                # PV = actual reading
                # SP = goal position
                t, PV, SP = yield MV

                # PID calculations
                e = SP - PV

                P = Kp*e
                I = I + Ki*e*(t - t_prev)
                D = Kd*(e - e_prev)/(t - t_prev)

                MV = MV_bar + P + I + D     

                # update stored data for next iteration
                e_prev = e
                t_prev = t

            return MV
        
        chessPositions = {
            'a8': [c(101.37), c(167.87), c(157.91), c(154.98), c(166.41), c(101.37), c(156.74), c(176.07), c(184.57), c(166.41)],
            'a7': [c(108.98), c(157.03), c(156.74), c(171.39), c(166.41), c(108.98), c(150.29), c(157.62), c(170.51), c(166.41)],
            'a6': [c(115.72), c(156.74), c(136.52), c(150.59), c(166.41), c(115.72), c(148.54), c(155.86), c(179.88), c(166.41)],
            'a5': [c(122.17), c(150.88), c(135.94), c(159.38), c(166.41), c(122.17), c(137.40), c(138.87), c(166.70), c(166.41)],
            'a4': [c(125.10), c(130.08), c(96.09), c(132.42), c(166.41), c(125.10), c(131.84), c(125.10), c(160.84), c(166.41)],
            'a3': [c(128.61), c(128.91), c(102.54), c(141.50), c(166.41), c(128.61), c(123.93), c(108.69), c(154.10), c(166.41)],
            'a2': [c(130.96), c(111.62), c(71.19), c(127.15), c(166.41), c(130.96), c(108.40), c(67.97), c(131.84), c(166.41)],
            'a1': [c(133.59), c(106.93), c(56.84), c(134.47), c(166.41), c(133.59), c(101.95), c(57.13), c(133.01), c(166.41)],
            'b8': [c(108.40), c(164.94), c(168.16), c(166.70), c(166.41), c(108.40), c(162.89), c(181.35), c(183.40), c(166.41)],
            'b7': [c(119.24), c(164.06), c(154.39), c(153.22),c (166.41), c(119.24), c(155.86), c(171.39), c(179.00), c(166.41)],
            'b6': [c(123.05), c(157.91), c(151.76), c(160.84), c(166.41), c(123.05), c(150.88), c(163.77), c(179.00), c(166.41)],
            'b5': [c(129.79), c(154.98), c(138.28), c(149.12), c(166.41), c(129.79), c(147.36), c(148.24), c(171.39), c(166.41)],
            'b4': [c(133.59), c(141.21), c(116.60), c(142.38), c(166.41), c(133.59), c(137.11), c(128.03), c(158.79), c(166.41)],
            'b3': [c(135.35), c(130.08), c(107.23), c(147.07), c(166.41), c(135.35), c(130.96), c(119.53), c(159.08), c(166.41)],
            'b2': [c(137.11), c(133.01), c(96.97), c(140.92), c(166.41), c(137.11), c(119.53), c(95.21), c(149.41), c(166.41)],
            'b1': [c(138.57), c(129.20), c(83.79), c(141.50), c(166.41), c(138.57), c(109.57), c(78.22), c(146.48), c(166.41)],
            'c8': [c(123.63), c(170.80), c(174.90), c(162.30), c(166.41), c(123.63), c(167.58), c(187.79), c(185.45), c(166.41)],
            'c7': [c(131.84), c(167.58), c(171.39), c(168.75), c(166.41), c(131.84), c(152.93), c(169.92), c(175.78), c(166.41)],
            'c6': [c(135.35), c(158.79), c(155.86), c(156.45), c(166.41), c(135.35), c(148.54), c(160.84), c(173.73), c(166.41)],
            'c5': [c(139.75), c(141.80), c(131.84), c(153.52), c(166.41), c(139.75), c(143.26), c(150.59), c(171.97), c(166.41)],
            'c4': [c(138.28), c(142.09), c(125.68), c(153.81), c(166.41), c(138.28), c(144.43), c(142.09), c(170.80), c(166.41)],
            'c3': [c(141.21), c(126.56), c(91.11), c(136.23), c(166.41), c(141.21), c(132.71), c(122.17), c(160.84), c(166.41)],
            'c2': [c(142.97), c(120.12), c(96.39), c(147.07), c(166.41), c(142.97), c(128.32), c(119.82), c(168.75), c(166.41)],
            'c1': [c(145.02), c(122.46), c(71.48), c(146.48), c(166.41), c(145.02), c(106.05), c(70.02), c(135.94), c(166.41)],
            'd8': [c(141.21), c(172.27), c(179.88), c(159.96), c(166.41), c(141.21), c(165.53), c(179.30), c(162.60), c(166.41)],
            'd7': [c(144.43), c(171.68), c(150.59), c(150.29), c(166.41), c(144.43), c(158.79), c(180.47), c(185.45), c(166.41)],
            'd6': [c(144.14), c(165.53), c(157.62), c(162.60), c(166.41), c(144.14), c(152.34), c(163.18), c(177.00), c(166.41)],
            'd5': [c(149.41), c(141.80), c(131.84), c(153.52), c(166.41), c(149.41), c(143.26), c(150.59), c(171.97), c(166.41)],
            'd4': [c(149.71), c(142.09), c(125.68), c(153.81), c(166.41), c(149.71), c(144.43), c(142.09), c(170.80), c(166.41)],
            'd3': [c(150.29), c(126.56), c(91.11), c(136.23), c(166.41), c(150.29), c(132.71), c(122.17), c(160.84), c(166.41)],
            'd2': [c(150.88), c(120.12), c(96.39), c(147.07), c(166.41), c(150.88), c(128.32), c(119.82), c(168.75), c(166.41)],
            'd1': [c(150.29), c(122.46), c(71.48), c(146.48), c(166.41), c(150.29), c(106.05), c(70.02), c(135.94), c(166.41)],
            'e8': [c(164.65), c(172.27), c(179.88), c(159.96), c(166.41), c(164.65), c(165.53), c(179.30), c(162.60), c(166.41)],
            'e7': [c(161.43), c(171.68), c(150.59), c(150.29), c(166.41), c(161.43), c(158.79), c(180.47), c(185.45), c(166.41)],
            'e6': [c(161.72), c(165.53), c(157.62), c(162.60), c(166.41), c(161.72), c(152.34), c(163.18), c(177.00), c(166.41)],
            'e5': [c(156.45), c(141.80), c(131.84), c(153.52), c(166.41), c(156.45), c(143.26), c(150.59), c(171.97), c(166.41)],
            'e4': [c(156.15), c(142.09), c(125.68), c(153.81), c(166.41), c(156.15), c(144.43), c(142.09), c(170.80), c(166.41)],
            'e3': [c(155.57), c(126.56), c(91.11), c(136.23), c(166.41), c(155.57), c(132.71), c(122.17), c(160.84), c(166.41)],
            'e2': [c(154.98), c(120.12), c(96.39), c(147.07), c(166.41), c(154.98), c(128.32), c(119.82), c(168.75), c(166.41)],
            'e1': [c(155.57), c(122.46), c(71.48), c(146.48), c(166.41), c(155.57), c(106.05), c(70.02), c(135.94), c(166.41)],
            'f8': [c(182.23), c(170.80), c(174.90), c(162.30), c(166.41), c(182.23), c(167.58), c(187.79), c(185.45), c(166.41)],
            'f7': [c(174.02), c(167.58), c(171.39), c(168.75), c(166.41), c(174.02), c(152.93), c(169.92), c(175.78), c(166.41)],
            'f6': [c(170.51), c(158.79), c(155.86), c(156.45), c(166.41), c(170.51), c(148.54), c(160.84), c(173.73), c(166.41)],
            'f5': [c(166.11), c(141.80), c(131.84), c(153.52), c(166.41), c(166.11), c(143.26), c(150.59), c(171.97), c(166.41)],
            'f4': [c(167.58), c(142.09), c(125.68), c(153.81), c(166.41), c(167.58), c(144.43), c(142.09), c(170.80), c(166.41)],
            'f3': [c(164.65), c(126.56), c(91.11), c(136.23), c(166.41), c(164.65), c(132.71), c(122.17), c(160.84), c(166.41)],
            'f2': [c(162.89), c(120.12), c(96.39), c(147.07), c(166.41), c(162.89), c(128.32), c(119.82), c(168.75), c(166.41)],
            'f1': [c(160.84), c(122.46), c(71.48), c(146.48), c(166.41), c(160.84), c(106.05), c(70.02), c(135.94), c(166.41)],
            'g8': [c(197.46), c(164.94), c(168.16), c(166.70), c(166.41), c(197.46), c(162.89), c(181.35), c(183.40), c(166.41)],
            'g7': [c(186.62), c(164.06), c(154.39), c(153.22),c (166.41), c(186.62), c(155.86), c(171.39), c(179.00), c(166.41)],
            'g6': [c(182.81), c(157.91), c(151.76), c(160.84), c(166.41), c(182.81), c(150.88), c(163.77), c(179.00), c(166.41)],
            'g5': [c(176.07), c(154.98), c(138.28), c(149.12), c(166.41), c(176.07), c(147.36), c(148.24), c(171.39), c(166.41)],
            'g4': [c(172.27), c(141.21), c(116.60), c(142.38), c(166.41), c(172.27), c(137.11), c(128.03), c(158.79), c(166.41)],
            'g3': [c(170.51), c(130.08), c(107.23), c(147.07), c(166.41), c(170.51), c(130.96), c(119.53), c(159.08), c(166.41)],
            'g2': [c(168.75), c(133.01), c(96.97), c(140.92), c(166.41), c(168.75), c(119.53), c(95.21), c(149.41), c(166.41)],
            'g1': [c(167.29), c(129.20), c(83.79), c(141.50), c(166.41), c(167.29), c(109.57), c(78.22), c(146.48), c(166.41)],
            'h8': [c(204.49), c(167.87), c(157.91), c(154.98), c(166.41), c(204.49), c(156.74), c(176.07), c(184.57), c(166.41)],
            'h7': [c(196.88), c(157.03), c(156.74), c(171.39), c(166.41), c(196.88), c(150.29), c(157.62), c(170.51), c(166.41)],
            'h6': [c(190.14), c(156.74), c(136.52), c(150.59), c(166.41), c(190.14), c(148.54), c(155.86), c(179.88), c(166.41)],
            'h5': [c(183.69), c(150.88), c(135.94), c(159.38), c(166.41), c(183.69), c(137.40), c(138.87), c(166.70), c(166.41)],
            'h4': [c(180.76), c(130.08), c(96.09), c(132.42), c(166.41), c(180.76), c(131.84), c(125.10), c(160.84), c(166.41)],
            'h3': [c(177.25), c(128.91), c(102.54), c(141.50), c(166.41), c(177.25), c(123.93), c(108.69), c(154.10), c(166.41)],
            'h2': [c(174.9), c(111.62), c(71.19), c(127.15), c(166.41), c(174.9), c(108.40), c(67.97), c(131.84), c(166.41)],
            'h1': [c(172.27), c(106.93), c(56.84), c(134.47), c(166.41), c(172.27), c(101.95), c(57.13), c(133.01), c(166.41)],
            'default': [c(152.34), c(188.09), c(150.88), c(89.06), c(0.00)],
            'out': [c(206.84), c(110.45), c(56.54), c(117.19), c(45)],
            'stretch': [512,c(155.86),c(54.20),c(250.78),c(0)],
            'test': [c(150), c(153.22), c(126.86), c(130.08), c(0)]
        }

        close = 133.47
        open = 185

        def moveToBeforeWithout(loc: str, speed=60):
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MOVING_SPEED, speed)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, int(chessPositions[loc][0]))
            time.sleep(5)

            positions = chessPositions[loc]

            second_difference = (d(positions[1]) - d(positions[6])) / 2
            second_speed = second_difference / 0.111
            third_difference = (d(positions[2]) - d(positions[7])) / 2
            third_speed = third_difference / 0.111
            fourth_difference = (d(positions[3]) - d(positions[8])) / 2
            fourth_speed = fourth_difference / 0.111

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MOVING_SPEED, int(second_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, int(positions[1]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MOVING_SPEED, int(third_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, int(positions[2]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MOVING_SPEED, int(fourth_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, int(positions[3]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))

            time.sleep(3)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVING_SPEED, 59)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))
            time.sleep(5)

        def moveToBeforeWith(loc: str, speed=60):
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MOVING_SPEED, speed)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, int(chessPositions[loc][0]))
            time.sleep(5)

            positions = chessPositions[loc]

            second_difference = (d(positions[1]) - d(positions[6])) / 2
            second_speed = second_difference / 0.111
            third_difference = (d(positions[2]) - d(positions[7])) / 2
            third_speed = third_difference / 0.111
            fourth_difference = (d(positions[3]) - d(positions[8])) / 2
            fourth_speed = fourth_difference / 0.111

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MOVING_SPEED, int(second_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, int(positions[1]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MOVING_SPEED, int(third_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, int(positions[2]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MOVING_SPEED, int(fourth_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, int(positions[3]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

            time.sleep(3)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVING_SPEED, 59)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))
            time.sleep(5)

        def moveToAfterWithout(loc: str, speed=60):
            servo_id_reverse = [4, 3, 2, 1]
            chess_position_reverse = chessPositions[loc][5:-1].copy()
            chess_position_reverse.reverse()

            servo_id_reverse_test = [4, 3, 2]
            chess_position_reverse_test = chessPositions[loc][6:-1].copy()
            chess_position_reverse_test.reverse()

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MOVING_SPEED, speed)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, int(chessPositions[loc][5]))
            time.sleep(5)

            # decimal / 2 / 0.111

            positions = chessPositions[loc]

            second_difference = (d(positions[6]) - d(positions[1])) / 2
            second_speed = second_difference / 0.111
            third_difference = (d(positions[7]) - d(positions[2])) / 2
            third_speed = third_difference / 0.111
            fourth_difference = (d(positions[8]) - d(positions[3])) / 2
            fourth_speed = fourth_difference / 0.111

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MOVING_SPEED, int(second_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, int(positions[6]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MOVING_SPEED, int(third_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, int(positions[7]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MOVING_SPEED, int(fourth_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, int(positions[8]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))

            time.sleep(3)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVING_SPEED, 59)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))
            time.sleep(5)

        def moveToAfterWith(loc: str, speed=60):
            servo_id_reverse = [4, 3, 2, 1]
            chess_position_reverse = chessPositions[loc][5:-1].copy()
            chess_position_reverse.reverse()

            servo_id_reverse_test = [4, 3, 2]
            chess_position_reverse_test = chessPositions[loc][6:-1].copy()
            chess_position_reverse_test.reverse()

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MOVING_SPEED, speed)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, int(chessPositions[loc][5]))

            time.sleep(5)

            # decimal / 2 / 0.111

            positions = chessPositions[loc]

            second_difference = (d(positions[6]) - d(positions[1])) / 2
            second_speed = second_difference / 0.111
            third_difference = (d(positions[7]) - d(positions[2])) / 2
            third_speed = third_difference / 0.111
            fourth_difference = (d(positions[8]) - d(positions[3])) / 2
            fourth_speed = fourth_difference / 0.111

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MOVING_SPEED, int(second_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, int(positions[6]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MOVING_SPEED, int(third_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, int(positions[7]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MOVING_SPEED, int(fourth_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, int(positions[8]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

            time.sleep(3)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_MOVING_SPEED, 59)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))
            time.sleep(5)

        def moveTo(before, after):
            positions_before = chessPositions[before]
            positions_after = chessPositions[after]

            first_difference = (d(positions_after[0]) - d(positions_before[0])) / 2
            first_speed = first_difference / 0.111
            second_difference = (d(positions_after[1]) - d(positions_before[1])) / 2
            second_speed = second_difference / 0.111
            third_difference = (d(positions_after[2]) - d(positions_before[2])) / 2
            third_speed = third_difference / 0.111
            fourth_difference = (d(positions_after[3]) - d(positions_before[3])) / 2
            fourth_speed = fourth_difference / 0.111

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MOVING_SPEED, int(second_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, int(positions_after[0]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MOVING_SPEED, int(second_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, int(positions_after[1]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MOVING_SPEED, int(third_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, int(positions_after[2]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MOVING_SPEED, int(fourth_speed))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, int(positions_after[3]))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))

        def moveOut(speed=100):
            servo_ids = [1, 2, 3, 4]
            for servo_id, position in zip(servo_ids, chessPositions['out'][:4]):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))
                # time.sleep(1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))
            time.sleep(5)

        def moveToDefault(speed=60):
            servo_ids = [1, 2, 3, 4]
            for servo_id, position in zip(servo_ids, chessPositions["default"][:4]):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))
                # time.sleep(1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))
            time.sleep(5)
        
        desired_speed = 81

        received = data.location.split()


        if received[2] == "m" or received[2] == "=m":
            moveToDefault()
            moveToBeforeWithout(received[0])
            moveToAfterWithout(received[0])
            moveToBeforeWith(received[0])
            moveToBeforeWith(received[1])
            moveToAfterWith(received[1])
            moveToBeforeWithout(received[1])
            # moveToDefault()
        elif received[2] == "x" or received[2] == "=x":
            moveToBeforeWithout(received[1])
            moveToAfterWithout(received[1])
            moveToBeforeWith(received[1])
            moveOut()
            moveToBeforeWithout(received[0])
            moveToAfterWithout(received[0])
            moveToBeforeWithout(received[0])
            moveToBeforeWith(received[1])
            moveToAfterWith(received[1])
            moveToBeforeWithout(received[1])
            # moveToDefault()
        elif received[2] == "default":
            moveToDefault()
        elif received[2] == "justonemove":
            moveToBeforeWithout(received[1])
            moveToAfterWithout(received[1])
            moveToBeforeWith(received[1])
        else:
            print("Error!")

    def get_present_pos(self, req, res):
        res.positions = []

        for servo_id in req.ids:
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)
            print("Present Position of ID %s = %s" % (servo_id, dxl_present_position))
            res.positions.append(int(dxl_present_position))

            self.dxl_present_positions[servo_id] = int(dxl_present_position)

        return res

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