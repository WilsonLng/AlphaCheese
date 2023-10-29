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