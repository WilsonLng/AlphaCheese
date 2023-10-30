import chess
import stockfish
import numpy as np
import os


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



if __name__ == "__main__":

    # 0. Initialisation
    # stockfish_path = "C:\\Users\\Nikodem Jan Dudek\\OneDrive\\Documents\\Robotics\\Group project\\Downloaded\\stockfish\\stockfish-windows-x86-64-avx2.exe"
    stockfish_path = "/home/steven/stockfish/stockfish-ubuntu-x86-64-avx2"
    stockfish = stockfish.Stockfish(stockfish_path)
    stockfish.set_depth(20)
    stockfish.set_skill_level(10)

    lines = []
    f = open("memory.txt", "r") # Too initiate a new game save an initial fen as a first only line
    for line in f:
        lines.append(line.strip())
    f.close()

    fen_loop = lines[-1]

    running = True
    while running:
        
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

        #positionwbe_2 = vision_to_positionwbe()
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

        index_fig0 = chessindex_to_index(suggested_move[0:2])
        index_fig1 = chessindex_to_index(suggested_move[2:4])

        position_3 = position_2.copy()
        position_3[index_fig0] = "-"
        
        if((position_2[index_fig1][0] == "-") and (len(suggested_move) == 4)):
            # 3a. Black executed a move of a figure.
            print("Input to the robotic arm: ", suggested_move[0:2] + " " + suggested_move[2:4])
            position_3[index_fig1] = position_2[index_fig0][0]

        elif((position_2[index_fig1][0] != "-") and (len(suggested_move) == 4)):
            # 3b. Black executed a capture.
            print("Input to the robotic arm: ", suggested_move[0:2] + "x" + suggested_move[2:4])
            position_3[index_fig1] = position_2[index_fig0][0]

        elif((position_2[index_fig1][0] == "-") and (len(suggested_move) == 5)):
            # 3c. Black executed a move of a figure with promotion
            print("Input to the robotic arm: ", suggested_move[0:2] + " " + suggested_move[2:4] + "=" + suggested_move[4])
            position_3[index_fig1] = suggested_move[4]

        elif((position_2[index_fig1][0] != "-") and (len(suggested_move) == 5)):
            # 3d. Black executed a capture with promotion
            print("Input to the robotic arm: ", suggested_move[0:2] + "x" + suggested_move[2:4] + "=" + suggested_move[4])
            position_3[index_fig1] = suggested_move[4]

        else:
            raise ValueError("Piece or suggested move issue")



        player = "w"
        fm = fm + 1
        fen_3 = position_to_fen(position_3, player, castling, ep, hm, fm)


        fen_loop = fen_3
        running = False

    print(fen_to_position(fen_loop), fen_loop)

    f = open('memory.txt', 'a')
    f.write("\n" + fen_loop)
    f.close()
    
"""
Position templates:
Note there are three ways of describing position from complex to simple:
- fen,
- position,
- positionwbe,
- positionbin

fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR b - - 0 1"
position = np.array([
    ["r", "n", "b", "q", "k", "b", "n", "r"],
    ["p", "p", "p", "p", "p", "p", "p", "p"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["P", "P", "P", "P", "P", "P", "P", "P"],
    ["R", "N", "B", "Q", "K", "B", "N", "R"]
    ])
positionwbe = np.array([
    ["b", "b", "b", "b", "b", "b", "b", "b"],
    ["b", "b", "b", "b", "b", "b", "b", "b"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["w", "w", "w", "w", "w", "w", "w", "w"],
    ["w", "w", "w", "w", "w", "w", "w", "w"]
    ])
positionbin = np.array([
    [1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1]          
])    
position_empty = np.array([
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"]          
])
"""
"""
TO DO:
-/ captured and no pieces from fen [Jennifer]
-/ promotion white
-/ promotion black
-/ input to robotic arm
- en passant (optional)
- draw
    - 50 rule
    - stalemate
    - dead position
    - mutual agreement
    - threefold repetition
- check
- checkmate
- castling (optional)

"""