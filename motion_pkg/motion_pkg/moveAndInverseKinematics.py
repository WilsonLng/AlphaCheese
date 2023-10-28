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
        super().__init__('moveAndInverseKinematics')
        self.subscriber_ = self.create_subscription(OneMove, 'which_position', self.set_goal_pos_callback, 10)
        self.service_ = self.create_service(GetPosition, 'get_position', self.get_present_pos)
        self.previous_message = None
        self.dxl_present_position = {}
        self.get_logger().info("Robot")

    def set_goal_pos_callback(self, data):
        def c(val):
            return val * 1023 / 300
        
        def anglesProduce(position: str, yf):
            R = 3
            d = 0.8                         
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

            def findYawRad(pos): #
                ver = int(pos[1])
                hor = hordict[pos[0]]
                degree = math.degrees(math.atan((side*(hor - 5) + (side/2)) / (R + d + side*(ver - 1) + (side/2))))
                print("degree= ", degree)
                return 150 - degree

            xf = findXf(position)

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
                while thetas[i] < 0:
                    thetas[i] = thetas[i] + 360
                while thetas[i] > 360:
                    thetas[i] = thetas[i] - 360

            for i in range(2, len(thetas)):
                thetas[i] = thetas[i] - 180

            thetas[0] = thetas[0] + 60
            thetas[2] = 240 - (3/5)*thetas[2]
            thetas[4] = thetas[4] + 60

            return [c(findYawRad(position)), c(thetas[0]), c(thetas[2]), c(thetas[4])]
        

        def moveToBefore(loc: str, speed=81):
            chessPosition = anglesProduce(loc, -3)
            chessPosition.append(c(50))
            print('Before: ', chessPosition)
            for servo_id, position in zip(servo_ids, chessPosition):
                print("Set Goal Position of ID before %s = %s, %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

        def moveToAfter(loc: str, speed=81):
            chessPosition = anglesProduce(loc, -9)
            chessPosition.append(c(0))
            print('After: ', chessPosition)
            for servo_id, position in zip(servo_ids, chessPosition):
                print("Set Goal Position of ID after %s = %s, %s" % (servo_id, int(position), position*300/1023))
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

        def autoMoveAndOffset(loc, speed=81):
            chessPosition = anglesProduce(loc, -5)
            chessPosition.append(c(0))
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

            Kp, Ki, Kd = 0.1, 0.01, 0.001

            for servo_id in range(2, 5):
                # ID is servo_id - 1 because starts with index 0
                goal = goal_values[servo_id - 1]

                pid = PIDController(Kp, Ki, Kd, goal)

                current_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)

                print("current_position: ", current_position)

                count = 0

                for _ in range(10):
                    control_signal = pid.update(current_position)

                    current_position += control_signal
                    print(f"{count} iteration current_position: ", current_position)

                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, speed)
                    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(current_position))
                    time.sleep(1)

                    count += 1

        servo_ids = [1, 2, 3, 4, 5]

        defaultPosition = [512,c(185),c(178),c(138),c(0)]

        desired_speed = 81

        # for servo_id, position in zip(servo_ids, defaultPosition):
        #     print("Set Goal Position of ID default %s = %s, %s" % (servo_id, int(position), position*300/1023))
        #     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_MOVING_SPEED, 81)
        #     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, int(position))

        autoMoveAndOffset("a1")

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
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

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