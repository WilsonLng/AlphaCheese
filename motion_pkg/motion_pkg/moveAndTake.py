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
            'a1': [],
            'a2': [],
            'a3': [],
            'a4': [],
            'a5': [],
            'a6': [],
            'a7': [],
            'a8': [],
            'b1': [],
            'b2': [],
            'b3': [],
            'b4': [],
            'b5': [],
            'b6': [],
            'b7': [],
            'b8': [c(111.91), c(172.85), c(157.62), c(155.86), c(0), c(111.91), c(160.25), c(174.02), c(174.90), c(0)],
            'c1': [],
            'c2': [],
            'c3': [],
            'c4': [],
            'c5': [],
            'c6': [c(137.11), c(167.87), c(157.91), c(167.87), c(0), c(137.11), c(156.15), c(162.01), c(177.54), c(0)],
            'c7': [],
            'c8': [],
            'd1': [],
            'd2': [],
            'd3': [],
            'd4': [c(149.41), c(151.76), c(135.64), c(162.30), c(0), c(149.41), c(142.97), c(147.95), c(177.54), c(0)],
            'd5': [c(146.78), c(158.50), c(147.07), c(157.62), c(0), c(146.78), c(150.29), c(153.81), c(171.39), c(0)],
            'd6': [],
            'd7': [c(146.78), c(164.65), c(148.83), c(149.12), c(0), c(146.78), c(155.57), c(166.70), c(170.21), c(0)],
            'd8': [],
            'e1': [],
            'e2': [],
            'e3': [],
            'e4': [c(158.20), c(153.52), c(140.63), c(171.39), c(0), c(158.20), c(141.80), c(140.04), c(165.82), c(0)],
            'e5': [c(158.50), c(155.57), c(142.09), c(156.15), c(45), c(158.20), c(150.29), c(160.55), c(179.00), c(45)],
            'e6': [],
            'e7': [c(161.43), c(171.68), c(163.48), c(154.39), c(45), c(161.43), c(166.11), c(183.98), c(186.04), c(45)],
            'e8': [],
            'f1': [],
            'f2': [],
            'f3': [],
            'f4': [],
            'f5': [],
            'f6': [c(169.92), c(167.29), c(171.09), c(182.23), c(0), c(169.29), c(150.88), c(166.11), c(182.81), c(0)],
            'f7': [],
            'f8': [],
            'g1': [],
            'g2': [],
            'g3': [],
            'g4': [],
            'g5': [],
            'g6': [],
            'g7': [],
            'g8': [c(193.65), c(176.07), c(177.83), c(173.44), c(0), c(193.65), c(168.75), c(182.81), c(186.04), c(0)],
            'h1': [],
            'h2': [],
            'h3': [],
            'h4': [],
            'h5': [],
            'h6': [],
            'h7': [],
            'h8': [],
            'default': [c(152.34), c(188.09), c(150.88), c(89.06), c(0.00), 512,c(185),c(178),c(138),c(0)],
            'out': [c(206.84), c(110.45), c(56.54), c(117.19), c(45)],
            'stretch': [512,c(155.86),c(54.20),c(250.78),c(0)],
            'test': [c(150), c(153.22), c(126.86), c(130.08), c(0)]
        }

        close = 133.47
        open = 166.70

        def moveToBeforeWithout(loc: str, speed=64):
            servo_ids = [1, 2, 3, 4]
            for servo_id, position in zip(servo_ids, chessPositions[loc][:4]):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))
            time.sleep(1)

        def moveToBeforeWith(loc: str, speed=64):
            servo_ids = [1, 2, 3, 4]
            for servo_id, position in zip(servo_ids, chessPositions[loc][:4]):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))
            time.sleep(1)

        def moveToAfterWithout(loc: str, speed=64):
            servo_id_reverse = [4, 3, 2, 1]
            chess_position_reverse = chessPositions[loc][5:-1].copy()
            chess_position_reverse.reverse()
            for servo_id, position in zip(servo_id_reverse, chess_position_reverse):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(open)))
            time.sleep(1)

        def moveToAfterWith(loc: str, speed=64):
            servo_id_reverse = [4, 3, 2, 1]
            chess_position_reverse = chessPositions[loc][5:-1].copy()
            chess_position_reverse.reverse()
            for servo_id, position in zip(servo_id_reverse, chess_position_reverse):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(close)))
            time.sleep(1)
        
        # servo_ids = [1, 2, 3, 4, 5]

        desired_speed = 81

        # for servo_id, position in zip(servo_ids[1:], [(188.09), c(150.88), c(89.06), c(0.00)]):
        #     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, 81)
        #     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
        #     time.sleep(2)

        time.sleep(5)
        # time.sleep(8)    
        # for servo_id in servo_ids:
        #     dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)
        #     # print("Present Position of ID %s = %s %s" % (servo_id, dxl_present_position, dxl_present_position * 300 / 1023))

        # print("after default")

        # stretch()

        # autoMoveAndOffset("h1")


        # e7: [c(161.43), c(171.68), c(163.48), c(154.39), c(45), c(159.08), c(166.11), c(183.98), c(186.04), c(45)]
        # e5: [c(158.50), c(155.57), c(142.09), c(156.15), c(45), c(158.20), c(150.29), c(160.55), c(179.00), c(45)]

        moveToBeforeWithout("e7")

        time.sleep(5)
        
        moveToAfterWithout("e7")

        time.sleep(5)

        moveToBeforeWith("e7")

        time.sleep(5)

        moveToBeforeWith("e5")

        time.sleep(5)

        moveToAfterWith("e5")

        time.sleep(5)

        moveToBeforeWithout("e5")

        time.sleep(5)

        moveToAfterWithout("e5")

        time.sleep(5)
        
        moveToBeforeWith("e5")

        time.sleep(5)

        moveToAfterWith("e5")

        # time.sleep(3)

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