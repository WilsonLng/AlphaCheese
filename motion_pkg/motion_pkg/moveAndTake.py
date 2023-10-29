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

class PIDController:
    def __init__(self, Kp, Ki, Kd, goal):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.goal = goal
        self.prev_error = 0
        self.integral = 0

    def update(self, current_position):
        error = self.goal - current_position

        P = self.Kp * error

        self.integral += error
        I = self.Ki * self.integral

        D = self.Kd * (error - self.prev_error)

        control_signal = P + I + D

        self.prev_error = error

        return control_signal

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
            'a4': [c(184.0),c(136.8),c(139.2),c(162.0),c(50),c(184.0),c(158.8),c(146.8),c(167.9),c(50)],
            'a5': [c(175),c(129.2),c(78.81),c(113.67),c(0)],
            'a6': [],
            'a7': [c(161.43), c(171.68), c(163.48), c(154.39), c(45), c(159.08), c(166.11), c(183.98), c(186.04), c(45)],
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
            'c6': [c(165), c(133.59), c(96.39), c(132.4), c(46.58), c(165), c(125.1), c(108.69), c(145.02), c(0)],
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
            'e5': [c(158.50), c(155.57), c(142.09), c(156.15), c(45), c(158.20), c(150.29), c(160.55), c(179.00), c(45)],
            'e6': [c(151.17), c(116.60), c(114.84), c(150.29), c(0)],
            'e7': [c(161.43), c(171.68), c(163.48), c(154.39), c(45), c(161.43), c(166.11), c(183.98), c(186.04), c(45)],
            'e8': [],
            'f1': [],
            'f2': [],
            'f3': [],
            'f4': [],
            'f5': [c(143.26),c(130.37),c(89.36),c(121.29),c(0)],
            'f6': [c(142.38),c(127.73),c(121),c(158.79),c(50)],
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
            'h1': [c(98),c(147.1),c(159.1),c(165),c(47),c(98),c(149.4),c(138.9),c(143.9),c(42.77)],
            'h2': [],
            'h3': [],
            'h4': [],
            'h5': [],
            'h6': [],
            'h7': [],
            'h8': [],
            'default': [c(152.34), c(188.09), c(150.88), c(89.06), c(0.00), 512,c(185),c(178),c(138),c(0)],
            'out': [],
            'stretch': [512,c(155.86),c(54.20),c(250.78),c(0)],
            'test': [c(150), c(153.22), c(126.86), c(130.08), c(0)]
        }

        def moveToBeforeWithout(loc: str, speed=81):
            for servo_id, position in zip(servo_ids, chessPositions[loc][:5]):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)

        def moveToBeforeWith(loc: str, speed=81):
            servos_ids = [1, 2, 3, 4]
            for servo_id, position in zip(servo_ids, chessPositions[loc][:4]):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(0)))

        def moveToAfterWithout(loc: str, speed=81):
            servo_id_reverse = [5, 4, 3, 2, 1]
            chess_position_reverse = chessPositions[loc][5:].copy()
            chess_position_reverse.reverse()
            for servo_id, position in zip(servo_id_reverse, chess_position_reverse):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(0)))

        def moveToAfterWith(loc: str, speed=81):
            servo_id_reverse = [4, 3, 2, 1]
            chess_position_reverse = chessPositions[loc][5:-1].copy()
            chess_position_reverse.reverse()
            for servo_id, position in zip(servo_id_reverse, chess_position_reverse):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, int(c(45)))

        def moveToAfter1(loc: str, speed=81):
            values = chessPositions[loc][:5]
            received_values = []
            for i in range(len(values)):
                values[i] = d(values[i])
            for servo_id, position in zip(servo_ids, chessPositions[loc][:5]):
                print("Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(3)
            for servo_id in servo_ids:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)
                received_values.append(dxl_present_position)
                print("Present Position of ID before feedback %s = %s %s" % (servo_id, dxl_present_position, dxl_present_position * 300 / 1023))
            for i in range(len(received_values)):
                print(f"before received_value: {received_values[i]}")
                received_values[i] = d(received_values[i])
                print(f"after received_value: {received_values[i]}")
            time.sleep(3)
            for i in range(1, 4):
                error = abs(values[i] - received_values[i])
                while error > 0:
                    print("values: ", values[i], "and received_values", received_values[i], "of ", i)
                    pid = PIDController(1.0, 0.1, 0.0)

                    SP = values[i]

                    t = time.time()

                    PV = received_values[i]

                    new_angle = pid.update(t, PV, SP)

                    received_values[i] = new_angle

                    error = abs(values[i] - received_values[i])

                    # if values[i] > received_values[i]:
                    #     received_values[i] += 1*15
                    # elif values[i] < received_values[i]:
                    #     received_values[i] -= 1*15
                    print("receive value after change of ", i+1, ": ", received_values[i])
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, i+1, ADDR_MOVING_SPEED, speed)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, i+1, ADDR_GOAL_POSITION, int(c(received_values[i])))
                    received_values = []
                    for servo_id in servo_ids:
                        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)
                        received_values.append(d(dxl_present_position))
                    print("Present Position of ID after feedback %s = %s %s" % (servo_id, dxl_present_position, dxl_present_position * 300 / 1023))

        def autoMoveAndOffset1(loc, speed=81):
            chessPositions = chessPositions[loc][:5]
            goal_values = chessPositions     # all values are in decimal
            received_values = []            # all values are in decimal
            print("autoMoveAndOffset Goal_values: ", goal_values)
            for servo_id, position in zip(servo_ids, chessPositions):
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)
            time.sleep(5)
            for i in range(len(servo_ids)):
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, i+1, ADDR_PRESENT_POSITION)
                received_values.append(dxl_present_position)
            print("autoMoveAndOffset Received_values: ", received_values)
            time.sleep(5)

            error = [i - j for i, j in zip(goal_values, received_values)]
            print("autoMoveAndOffset error: ", error)

            offset_values = goal_values
            for i in range(1, len(error) - 2):
                if abs(error[i]) > 5:
                    adjust = min(abs(error[i]*10), 20)
                    if i != 2:
                        offset_values[i] = offset_values[i] + adjust
                    else:
                        offset_values[i] = offset_values[i] - adjust

            print("autoMoveAndOffset offset_values: ", offset_values)
            print("chessPositions values          : ", chessPositions)

            time.sleep(5)
            for servo_id, position in zip(servo_ids, chessPositions):
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)

            print("Completed----------------------------------")

        def stretch():
            for servo_id, position in zip(servo_ids, chessPositions["stretch"][:5]):
                # print("Stretch Set Goal Position of ID %s = %s %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, 81)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(3)
            for servo_id in servo_ids:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)
                # print("Stretch Present Position of ID before feedback %s = %s %s" % (servo_id, dxl_present_position, dxl_present_position * 300 / 1023))

        def autoMoveAndOffset(loc, speed=81):
            chessPosition = chessPositions["h1"][:5]
            goal_values = chessPosition.copy()    # all values are in decimal
            received_values = []                  # all values are in decimal
            print("Goal_values: ", goal_values)
            for servo_id, position in zip(servo_ids, chessPosition):
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
                time.sleep(1)
            time.sleep(5)
            for i in range(len(servo_ids)):
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, i+1, ADDR_PRESENT_POSITION)
                received_values.append(dxl_present_position)
            print("Received_values: ", received_values)
            time.sleep(5)

            Kp, Ki, Kd = 5, 0.01, 0.001

            for servo_id in range(2, 5):
                # ID is servo_id - 1 because starts with index 0
                goal = goal_values[servo_id - 1]

                pid = PIDController(Kp, Ki, Kd, goal)

                current_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)

                print("current_position: ", current_position)

                count = 0

                for _ in range(50):
                    control_signal = pid.update(current_position)

                    current_position += control_signal
                    print(f"{count} control_signal: ", control_signal)
                    print(f"{count} iteration current_position: ", current_position)

                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(current_position))
                    time.sleep(0.5)

                    current_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)

                    count += 1
        
        servo_ids = [1, 2, 3, 4, 5]

        desired_speed = 81

        for servo_id, position in zip(servo_ids[1:], [c(188.09), c(150.88), c(89.06), c(0.00)]):
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, 81)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))
            time.sleep(2)

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