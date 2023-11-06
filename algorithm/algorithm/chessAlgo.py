import chess
import stockfish
import numpy as np
import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

positionwbe_2 = np.array([
                    ["b", "b", "b", "b", "b", "b", "b", "b"],
                    ["b", "b", "b", "b", "b", "b", "b", "b"],
                    ["-", "-", "-", "-", "-", "-", "-", "-"],
                    ["-", "-", "-", "-", "-", "-", "-", "-"],
                    ["-", "-", "-", "-", "-", "-", "-", "-"],
                    ["-", "-", "-", "-", "-", "w", "-", "-"],
                    ["w", "w", "w", "w", "w", "w", "w", "w"],
                    ["w", "w", "w", "w", "w", "w", "-", "w"]
                    ])

class ChessGameNode(Node):
    def __init__(self):
        super().__init__('chess_game_node')
        self.subscription = self.create_subscription(
            String,
            'chessboard_state',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'which_position', 10)
        self.subscription  # prevent unused variable warning

        # Initialize Stockfish engine
        self.stockfish_path = "/home/steven/stockfish/stockfish-ubuntu-x86-64-avx2"
        self.stockfish = stockfish.Stockfish(self.stockfish_path)
        self.stockfish.set_depth(20)
        self.stockfish.set_skill_level(10)

        # Load the last game state
        self.lines = []
        with open("memory.txt", "r") as f:
            for line in f:
                self.lines.append(line.strip())

        self.fen_loop = self.lines[-1]
        self.previous_state = None
        print("Initialisation done")

    def listener_callback(self, msg):
        current_state = np.array(json.loads(msg.data))
        print("Data received:")
        print(current_state)

        # Convert the received chessboard state to the appropriate format
        # and perform the logic for the white player's turn
        # ...
        
        if self.previous_state is not None:  # Ensure previous state exists
            if not np.array_equal(current_state, self.previous_state):
                # A change has been detected; process the new state and suggest a move
                self.get_logger().info('Change detected in chessboard state.')

                # 1. Beginning of a turn.
                fen_1 = fen_loop
                position_1 = fen_to_position(fen_1)
                positionwbe_1 = position_to_positionwbe(position_1)
                positionbin_1 = position_or_positionwbe_to_positionbin(position_1)
                castling = fen_1.split()[2]
                ep = fen_1.split()[3]  # En passant target square
                hm = int(fen_1.split()[4])  # Halfmove clock
                fm = int(fen_1.split()[5])  # Fullmove number
                
                # 2. White plays.

                positionwbe_2 = vision_to_positionwbe(positionwbe_1, current_state)
                # positionwbe_2 = np.array([
                #     ["b", "b", "b", "b", "b", "b", "b", "b"],
                #     ["b", "b", "b", "b", "b", "b", "b", "b"],
                #     ["-", "-", "-", "-", "-", "-", "-", "-"],
                #     ["-", "-", "-", "-", "-", "-", "-", "-"],
                #     ["-", "-", "-", "-", "-", "-", "-", "-"],
                #     ["-", "-", "-", "-", "-", "w", "-", "-"],
                #     ["w", "w", "w", "w", "w", "w", "w", "w"],
                #     ["w", "w", "w", "w", "w", "w", "-", "w"]
                #     ])
                positionbin_2 = position_or_positionwbe_to_positionbin(positionwbe_2)

                no_chesspieces = np.sum(positionbin_2)
                position_2 = position_1.copy()

                if(np.sum(positionbin_2) == np.sum(positionbin_1)):
                    # 2a. White executed a move of a figure.
                    diff = (positionbin_2 - positionbin_1)
                    index_fig0 = np.where(diff == -1)
                    index_fig1 = np.where(diff == 1)
                    position_2[index_fig0] = "-"
                    position_2[index_fig1] = position_1[index_fig0][0]

                elif(np.sum(positionbin_2) + 1 == np.sum(positionbin_1)):
                    # 2b. White executed a capture.
                    diff = (positionbin_2 - positionbin_1)
                    index_fig0 = np.where(diff == -1)
                    diff = ((positionwbe_2 != positionwbe_1) & (positionwbe_2 == "w"))
                    index_fig1 = np.where(diff == True)
                    position_2[index_fig0] = "-"
                    position_2[index_fig1] = position_1[index_fig0][0]

                else:
                    print(np.sum(positionbin_2), np.sum(positionbin_1))
                    raise ValueError("No. chesspieces does not match")
                
                for i in range(8):  # White promotion
                    if(position_2[0, i] == "P"):
                        position_2[0, i] == "Q"

                player = "b"
                fen_2 = position_to_fen(position_2, player, castling, ep, hm, fm)
                
                # 3. Black plays.
                suggested_move = suggest_move(fen_2, stockfish)
                # suggested_move = self.determine_move(current_state, stockfish)
                if suggested_move:
                    index_fig0 = chessindex_to_index(suggested_move[0:2])
                    index_fig1 = chessindex_to_index(suggested_move[2:4])

                    position_3 = position_2.copy()
                    position_3[index_fig0] = "-"
                    
                    if((position_2[index_fig1][0] == "-") and (len(suggested_move) == 4)):
                        # 3a. Black executed a move of a figure.
                        formatted_move = suggested_move[0:2] + " " + suggested_move[2:4] + " m"
                        self.get_logger().info('Suggested Move: "%s"' % formatted_move)
                        self.publish_move(suggested_move)
                        print("Input to the robotic arm: ", formatted_move)
                        position_3[index_fig1] = position_2[index_fig0][0]

                    elif((position_2[index_fig1][0] != "-") and (len(suggested_move) == 4)):
                        # 3b. Black executed a capture.
                        formatted_move = suggested_move[0:2] + " " + suggested_move[2:4] + " x"
                        self.get_logger().info('Suggested Move: "%s"' % formatted_move)
                        self.publish_move(suggested_move)
                        print("Input to the robotic arm: ", formatted_move)
                        position_3[index_fig1] = position_2[index_fig0][0]

                    elif((position_2[index_fig1][0] == "-") and (len(suggested_move) == 5)):
                        # 3c. Black executed a move of a figure with promotion
                        formatted_move = suggested_move[0:2] + " " + suggested_move[2:4] + " =m" + suggested_move[4]
                        self.get_logger().info('Suggested Move: "%s"' % formatted_move)
                        self.publish_move(suggested_move)
                        print("Input to the robotic arm: ", formatted_move)
                        position_3[index_fig1] = suggested_move[4]

                    elif((position_2[index_fig1][0] != "-") and (len(suggested_move) == 5)):
                        # 3d. Black executed a capture with promotion
                        formatted_move = suggested_move[0:2] + " " + suggested_move[2:4] + " =x" + suggested_move[4]
                        self.get_logger().info('Suggested Move: "%s"' % formatted_move)
                        self.publish_move(suggested_move)
                        print("Input to the robotic arm: ", formatted_move)
                        position_3[index_fig1] = suggested_move[4]

                    else:
                        raise ValueError("Piece or suggested move issue")

                    player = "w"
                    fm = fm + 1
                    fen_3 = position_to_fen(position_3, player, castling, ep, hm, fm)

                    fen_loop = fen_3

    def publish_move(self, move):
        msg = String()
        msg.data = move
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing move to which_position: "%s"' % move)

    def determine_move(self, fen_position, stockfish_engine):
        engine = stockfish_engine
        engine.set_fen_position(fen_position)
        return engine.get_best_move()

def fen_to_position(fen_num):
    position = np.empty((8, 8), dtype=object)
    fen_num = fen_num[:-10]
    fen = fen_num.split('/')
    RN = 0  # row number
    for element in fen:
        RN = RN + 1
        CL = 0
        for alphabet in element:
            if alphabet.isalpha():
                CL = CL + 1
                position[RN - 1, CL - 1] = alphabet
            else:
                num = int(alphabet)
                for i in range(num):
                    CL = CL + 1
                    position[RN - 1, CL - 1] = "-"
    return position

def position_to_fen(position, player: str, castling: str, ep: str, hm: int, fm: int): 
    fen = ""
    for row in position:
        empty_count = 0
        for square in row:
            if square == '-':
                empty_count += 1
            else:
                if empty_count > 0:
                    fen += str(empty_count)
                    empty_count = 0
                fen += square
        if empty_count > 0:
            fen += str(empty_count)
        fen += '/'

    fen = fen[:-1]  # remove the extra '/'
    fen = fen + " " + player + " " + castling + " " + ep + " " + str(hm) + " " + str(fm)
    return fen

def position_to_positionwbe(position):
    pos_empt = np.empty((8, 8), dtype=object)
    RN = 0  # row number
    for row in position:
        RN = RN + 1
        CL = 0
        for element in row:
            CL = CL + 1
            if element.isupper():
                pos_empt[RN-1, CL-1] = "w"
            elif element == "-":
                pos_empt[RN-1, CL-1] = "-" 
            elif element.islower():
                pos_empt[RN-1, CL-1] = "b"
    return pos_empt

def position_or_positionwbe_to_positionbin(position):
    pos_empt = np.empty((8, 8), dtype=object)
    RN = 0  # row number
    for row in position:
        RN = RN + 1
        CL = 0
        for element in row:
            CL = CL + 1
            if element != "-":
                pos_empt[RN-1, CL-1] = 1
            if element == "-":
                pos_empt[RN-1, CL-1] = 0  
    return pos_empt

def chessindex_to_index(chessindex):
    if len(chessindex) != 2:
        raise ValueError("Invalid input. Chess coordinates should be two characters (e.g., 'a1').")

    file_to_index = {
        'a': 0, 'b': 1, 'c': 2, 'd': 3,
        'e': 4, 'f': 5, 'g': 6, 'h': 7
    }

    rank = 8 - int(chessindex[1])
    file = file_to_index[chessindex[0]]

    if 0 <= rank <= 7 and 0 <= file <= 7:
        return (rank, file)
    else:
        raise ValueError("Invalid chess coordinates. Coordinates should be in the range 'a1' to 'h8'.")

def switch_players(fen):
    fen_parts = fen.split()

    if len(fen_parts) == 6 and fen_parts[1] in ("w", "b"):
        fen_parts[1] = "b" if fen_parts[1] == "w" else "w"
        new_fen = " ".join(fen_parts)
        return new_fen
    else:
        return None

def suggest_move(fen_position, stockfish):
    engine = stockfish
    engine.set_fen_position(fen_position)
    suggested_move = engine.get_best_move()
    return suggested_move

def vision_to_positionwbe(white_state, wbe):
    # Assuming white_state's shape is (8, 8) and it contains positions of the white pieces anywhere on the board.

    # Loop over all positions of the board.
    for rank in range(8):
        for file in range(8):
            current_piece = white_state[rank, file]
            # If there is a white piece in the white_state, update the wbe
            if current_piece == 'w':
                wbe[rank, file] = 'w'
            # If the current position is marked with white in the wbe but not in the white_state, clear it
            elif wbe[rank, file] == 'w' and current_piece == '-':
                wbe[rank, file] = '-'
    return wbe

#def is_checkmate_fen(fen):
#    board = chess.Board(fen)
#    return board.is_checkmate()

#def captured_pieces(fen):
#    fen = fen.split()[0]
#    white_captured = []
#    black_captured = []
#    for i in [Q, B, R, N]
#    for char in fen:
#        if char == 'Q':
#            count += 1

def no_chesspieces(fen):
    piece_counts = {
        'K': 0, 'Q': 0, 'R': 0, 'B': 0, 'N': 0, 'P': 0,
        'k': 0, 'q': 0, 'r': 0, 'b': 0, 'n': 0, 'p': 0
    }
    fen = fen.split()[0]
    for char in fen:
        if char in piece_counts:
            piece_counts[char] += 1
    total_count = sum(piece_counts.values())
    return total_count

def main(args=None):
    rclpy.init(args=args)

    chess_game_node = ChessGameNode()

    try:
        rclpy.spin(chess_game_node)  # spin() will block execution and keep the node alive
    except KeyboardInterrupt:
        pass  # Handle Ctrl-C gracefully

    # Clean up before shutting down.
    chess_game_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
