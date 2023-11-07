import numpy as np
import cv2 as cv
import json
from roboflow import Roboflow
import matplotlib.pyplot as plt
from shapely.geometry import LineString
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

chesspiece_delay = 0.5 # in seconds

chessboard_path = 'chessboard-ref.jpg'
chessboard_mapped_path = 'chessboard-mapped.jpg'
chesspiece_path = 'chesspiece-ref.jpg'

class TurnListener(Node):
    def __init__(self):
        super().__init__('turn_listener')
        self.subscription = self.create_subscription(
            String,
            'whose_turn',
            self.turn_listener_callback,
            10)
        self.white_is_playing = False  # Flag to check if it's white's turn to play

    def turn_listener_callback(self, msg):
        if msg.data.lower() == 'white is playing':
            self.white_is_playing = True
        elif msg.data.lower() == 'black is playing':
            self.white_is_playing = False
        else:
            self.get_logger().error(f"Unrecognized message on 'whose_turn': {msg.data}")

class ChessBoardPublisher(Node):
    def __init__(self):
        super().__init__('chessboard_publisher')
        self.publisher_ = self.create_publisher(String, 'chessboard_state', 10)

    def publish_board(self, chess_board):
        msg = String()
        # Convert the numpy array to a JSON string for publishing
        msg.data = json.dumps(chess_board.tolist())
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "\n%s"' % msg.data)

class Trapezium:
    def __init__(self, trapezium_x, y_coord, image_path):
        self.trapezium_x = trapezium_x
        self.y_coord = y_coord
        self.y11 = max(y_coord)
        self.y0 = min(y_coord)
        self.x0 = trapezium_x[0] 
        self.x1 = trapezium_x[1] 
        self.x2 = trapezium_x[2]
        self.x3 = trapezium_x[3] 
        self.intersection_points = []
        self.upper_coord = []
        self.lower_coord = []
        self.image_path = image_path

    def calculate_val_x(self, point1, point2, y):  
        x1, y1 = point1
        x2, y2 = point2

        if x1 == x2:
            return x1

        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
        x = (y - b) / m

        return round(x, 3)

    def plot_graph(self):
        img = cv.imread(self.image_path)
        
        upper_segments = np.linspace(self.x3, self.x2, 9)
        self.upper_coord = [(i, self.y11) for i in upper_segments]
        lower_segments = np.linspace(self.x0, self.x1, 9)
        self.lower_coord = [(i, self.y0) for i in lower_segments]

        horz_coord1 = [(min(self.trapezium_x), j) for j in self.y_coord]
        horz_coord2 = [(max(self.trapezium_x), j) for j in self.y_coord]

        for i in range(9):
            line1 = LineString([self.upper_coord[i], self.lower_coord[i]])
            for k in range(9):
                line2 = LineString([horz_coord1[k], horz_coord2[k]])
                intersection = line1.intersection(line2)
                if intersection:
                    # print("intersection: ", intersection)
                    if intersection.geom_type == 'Point':
                        self.intersection_points.append((intersection.x, intersection.y))
                        im_chessboard = cv.circle(img, (int(intersection.x), int(intersection.y)), 5, (0,0,255), -1)
                        # plt.plot(intersection.x, intersection.y, 'ro')

        # for i in range(9):
        #     plt.plot([self.upper_coord[i][0], self.lower_coord[i][0]], [self.upper_coord[i][1], self.lower_coord[i][1]], 'b--')

        # for i in range(9):
        #     plt.plot([horz_coord1[i][0], horz_coord2[i][0]], [horz_coord1[i][1], horz_coord2[i][1]], 'g--')

        # plt.show()
        cv.imshow("Boxes Mapped", im_chessboard)
        cv.imwrite(chessboard_mapped_path, im_chessboard)

    def find_pos(self, x1, y1, side_view=1):
        sorted_intersection_points = np.array(sorted(self.intersection_points, key=lambda p: (p[1], p[0]))).reshape(9, 9, 2)
        pos = np.zeros((8, 8), dtype=object)
        
        # for i in range(8):
        #         for j in range(8):
        #             pos[7 - i, j] = chr(j + 97) + str(i + 1)

        if side_view:
            # Generate the labels from h1 to a8
            for i in range(8):
                for j in range(8):
                    pos[i, j] = chr(104 - i) + str(j + 1)
        
        else:
            # Generate the labels from h8 to a1
            for i in range(8): 
                for j in range(8): 
                    pos[i, j] = chr(104 - j) + str(8 - i)

        # print("pos: ", pos)

        for j in range(len(sorted_intersection_points)-1):
            if y1 > sorted_intersection_points[j, 0, 1] and y1 < sorted_intersection_points[j + 1, 0, 1]:
                k = j

        if y1 < min(sorted_intersection_points[:, 0, 1]) or y1 > max(sorted_intersection_points[:, 0, 1]):
            # print("Coordinate outside the range of y")
            return 0
        
        elif y1 in sorted_intersection_points[:, 0, 1]:
            # print("Chess piece is placed on the edge of the squares")
            return 0
        
        else:
            x_pot = []
            for i in range(len(self.upper_coord)):
                point1 = self.upper_coord[i]
                point2 = self.lower_coord[i]
                x = self.calculate_val_x(point1, point2, y1)
                x_pot.append(x)
            
            for p in range(len(x_pot)-1):
                if x1 > float(x_pot[p]) and x1 < float(x_pot[p + 1]):
                    h = p
            if x1 < min(x_pot) or x1 > max(x_pot):
                # print("Coordinate outside the range of x")
                return 0
            elif x1 in x_pot:
                # print("Chess piece is placed on the edge of the squares")
                return 0
            else:
                # print('Chess piece position is:', pos[7-k,h])
                return pos[7-k,h]

def dict_to_board(chess_dict):
    # Initialize an 8x8 numpy array with empty strings
    chess_board = np.full((8, 8), "-", dtype=str)

    # Loop through the dictionary to populate the array
    for square, color in chess_dict.items():
        # Convert the square in chess notation to row and column in the numpy array
        col = ord(square[0]) - 97  # 'a' to 0, 'b' to 1, ..., 'h' to 7
        row = 8 - int(square[1])  # '1' to 7, '2' to 6, ..., '8' to 0

        # Convert the descriptive color to its shorthand ('w', 'b', or '-')
        if color == "white":
            value = 'w'
        elif color == "black":
            value = 'b'
        else:
            value = '-'

        # Place the shorthand color value in the corresponding position on the board
        chess_board[row, col] = value

    return chess_board

def board_to_dict(chess_board):
    # Create an empty dictionary to hold the square-color mappings
    chess_dict = {}
    
    # Rows correspond to the rank on the chess board (8 to 1)
    for row in range(8):
        # Columns correspond to the file on the chess board (a to h)
        for col in range(8):
            # Convert row and column to chess notation
            square = f"{chr(97 + col)}{8 - row}"
            
            # Get the value from the numpy array
            value = chess_board[row, col]
            
            # Convert the value to a descriptive word ("white", "black", or "empty")
            if value == 'w':
                color = "white"
            elif value == 'b':
                color = "black"
            else:
                color = "empty"
            
            # Add the square and its color to the dictionary
            chess_dict[square] = color

    return chess_dict

def findVertices(json_data,range_ymax = 15,range_ymin =5 ,range_xmax = 100, range_xmin=100):
    
    data = json_data

    # Extract points from data
    points = data["predictions"][0]["points"]
    
    first_edges = []
    sec_edges = []
    
    ##-----------------FIRST PART ( Y range )---------------------------------------------

    # Find the point with the minimum y value
    min_point = min(points, key=lambda point: point["y"])

    # Find the point with the maximum y value
    max_point = max(points, key=lambda point: point["y"])
    
    # Define the y value ranges
    y_max_range = (max_point["y"], max_point["y"] - range_ymax)
    y_min_range = (min_point["y"], min_point["y"] + range_ymin)

    # Filter points within the specified y value ranges
    points_within_y_max_range = [point for point in points if y_max_range[1] <= point["y"] <= y_max_range[0]]
    points_within_y_min_range = [point for point in points if y_min_range[0] <= point["y"] <= y_min_range[1]]

    # Sort the filtered points based on x-coordinate
    sorted_points_within_y_max_range = sorted(points_within_y_max_range, key=lambda point: point["x"])
    sorted_points_within_y_min_range = sorted(points_within_y_min_range, key=lambda point: point["x"])

    # Extract the required points
    min_x_within_y_max = sorted_points_within_y_max_range[0]
    max_x_within_y_max = sorted_points_within_y_max_range[-1]
    min_x_within_y_min = sorted_points_within_y_min_range[0]
    max_x_within_y_min = sorted_points_within_y_min_range[-1]

    # Create the 'edges' array by concatenating the points
    first_edges = [min_x_within_y_max, max_x_within_y_max, max_x_within_y_min, min_x_within_y_min]
    
    ## ----------------- SECOND PART (X range) -------------------------------

    # Find the point with the minimum x value
    min_point = min(points, key=lambda point: point["x"])

    # Find the point with the maximum x value
    max_point = max(points, key=lambda point: point["x"])
    
    print("min_point",min_point["x"])
    print("max_point",max_point["x"])
    
    # Find the mid point 
    mid_point = (min_point["x"]+max_point["x"])/2 
    print("mid_point",mid_point)

    # Define the x value ranges
    
    ## METHOD 1
    #  x_max_range = (max_point["x"], max_point["x"] - range_xmax)
    #  x_min_range = (min_point["x"], min_point["x"] + range_xmin)

    ## METHOD 2
    #  x_max_range = (max_point["x"],  (mid_point+range_xmax))
    #  x_min_range = (min_point["x"],  (mid_point-range_xmin))

    ## METHOD 3
    x_max_range = (max_point["x"],  (max_point["x"] - (max_point["x"]-mid_point)*0.5))
    x_min_range = (min_point["x"],  (min_point["x"] + (mid_point-min_point["x"])*0.5))
    

    # Filter points within the specified x value ranges
    points_within_x_max_range = [point for point in points if x_max_range[1] <= point["x"] <= x_max_range[0]]
    points_within_x_min_range = [point for point in points if x_min_range[0] <= point["x"] <= x_min_range[1]]

    # Sort the filtered points based on y-coordinate
    sorted_points_within_x_max_range = sorted(points_within_x_max_range, key=lambda point: point["y"])
    sorted_points_within_x_min_range = sorted(points_within_x_min_range, key=lambda point: point["y"])

    min_y_within_x_max = sorted_points_within_x_max_range[0]
    max_y_within_x_max = sorted_points_within_x_max_range[-1]
    min_y_within_x_min = sorted_points_within_x_min_range[0]
    max_y_within_x_min = sorted_points_within_x_min_range[-1]


    sec_edges = [max_y_within_x_min] + [max_y_within_x_max] + [min_y_within_x_max] + [min_y_within_x_min]

    
    ## ----------THIRD PART---------------------

    ### LEFT AND RIGHT LINE
    # Extract the first two points and the last two points of sec_edges
    first_two_points = [first_edges[0] , first_edges[3]]
    last_two_points = [first_edges[1] , first_edges[2]]
    

    # Extract x and y coordinates for the first and last two points
    x1_first, y1_first = zip(*[(point['x'], point['y']) for point in first_two_points])
    x2_first, y2_first = zip(*[(point['x'], point['y']) for point in last_two_points])

    ### TOP AND BOTTOM LINE
    # Extract the first two points and the last two points of sec_edges
    first_two_points = sec_edges[:2]
    last_two_points = sec_edges[2:]
    
    # Extract x and y coordinates for the first and last two points
    x1_sec, y1_sec = zip(*[(point['x'], point['y']) for point in first_two_points])
    x2_sec, y2_sec = zip(*[(point['x'], point['y']) for point in last_two_points])

    ### FIND INTERSECTION POINTS
    intersect_pts = []

    # compile lines 
    lines = [[x1_first, y1_first],[x1_sec, y1_sec],[x2_first, y2_first],[x2_sec, y2_sec],[x1_first, y1_first]]

    # Find the intersection point
    # Define the equations of the lines: y = m1 * x + b1, and y = m2 * x + b2
    for i in range(0,4):
        m1, b1 = np.polyfit(lines[i][0], lines[i][1], 1)  # Line 1
        m2, b2 = np.polyfit(lines[i+1][0], lines[i+1][1], 1)  # Line 2

        # Calculate the intersection point
        x_intersection = (b2 - b1) / (m1 - m2)
        y_intersection = m1 * x_intersection + b1

        #print(intersect_pts)
        intersect_pts += [[x_intersection,y_intersection]]

    
    return intersect_pts

# Run the main function
if __name__ == "__main__":
    main()

class ChessBoardPublisher(Node):
    def __init__(self):
        super().__init__('chessboard_publisher')
        self.publisher_ = self.create_publisher(String, 'chessboard_state', 10)

    def publish_board(self, chess_board):
        msg = String()
        # Convert the numpy array to a JSON string for publishing
        msg.data = json.dumps(chess_board.tolist())
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "\n%s"' % msg.data)

class Trapezium:
    def __init__(self, trapezium_x, y_coord, image_path):
        self.trapezium_x = trapezium_x
        self.y_coord = y_coord
        self.y11 = max(y_coord)
        self.y0 = min(y_coord)
        self.x0 = trapezium_x[0] 
        self.x1 = trapezium_x[1] 
        self.x2 = trapezium_x[2]
        self.x3 = trapezium_x[3] 
        self.intersection_points = []
        self.upper_coord = []
        self.lower_coord = []
        self.image_path = image_path

    def calculate_val_x(self, point1, point2, y):  
        x1, y1 = point1
        x2, y2 = point2

        if x1 == x2:
            return x1

        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
        x = (y - b) / m

        return round(x, 3)

    def plot_graph(self):
        img = cv.imread(self.image_path)
        
        upper_segments = np.linspace(self.x3, self.x2, 9)
        self.upper_coord = [(i, self.y11) for i in upper_segments]
        lower_segments = np.linspace(self.x0, self.x1, 9)
        self.lower_coord = [(i, self.y0) for i in lower_segments]

        horz_coord1 = [(min(self.trapezium_x), j) for j in self.y_coord]
        horz_coord2 = [(max(self.trapezium_x), j) for j in self.y_coord]

        for i in range(9):
            line1 = LineString([self.upper_coord[i], self.lower_coord[i]])
            for k in range(9):
                line2 = LineString([horz_coord1[k], horz_coord2[k]])
                intersection = line1.intersection(line2)
                if intersection:
                    # print("intersection: ", intersection)
                    if intersection.geom_type == 'Point':
                        self.intersection_points.append((intersection.x, intersection.y))
                        im_chessboard = cv.circle(img, (int(intersection.x), int(intersection.y)), 5, (0,0,255), -1)
                        # plt.plot(intersection.x, intersection.y, 'ro')

        # for i in range(9):
        #     plt.plot([self.upper_coord[i][0], self.lower_coord[i][0]], [self.upper_coord[i][1], self.lower_coord[i][1]], 'b--')

        # for i in range(9):
        #     plt.plot([horz_coord1[i][0], horz_coord2[i][0]], [horz_coord1[i][1], horz_coord2[i][1]], 'g--')

        # plt.show()
        cv.imshow("Boxes Mapped", im_chessboard)
        cv.imwrite(chessboard_mapped_path, im_chessboard)

    def find_pos(self, x1, y1, side_view=1):
        sorted_intersection_points = np.array(sorted(self.intersection_points, key=lambda p: (p[1], p[0]))).reshape(9, 9, 2)
        pos = np.zeros((8, 8), dtype=object)
        
        # for i in range(8):
        #         for j in range(8):
        #             pos[7 - i, j] = chr(j + 97) + str(i + 1)

        if side_view:
            # Generate the labels from h1 to a8
            for i in range(8):
                for j in range(8):
                    pos[i, j] = chr(104 - i) + str(j + 1)
        
        else:
            # Generate the labels from h8 to a1
            for i in range(8): 
                for j in range(8): 
                    pos[i, j] = chr(104 - j) + str(8 - i)

        # print("pos: ", pos)

        for j in range(len(sorted_intersection_points)-1):
            if y1 > sorted_intersection_points[j, 0, 1] and y1 < sorted_intersection_points[j + 1, 0, 1]:
                k = j

        if y1 < min(sorted_intersection_points[:, 0, 1]) or y1 > max(sorted_intersection_points[:, 0, 1]):
            # print("Coordinate outside the range of y")
            return 0
        
        elif y1 in sorted_intersection_points[:, 0, 1]:
            # print("Chess piece is placed on the edge of the squares")
            return 0
        
        else:
            x_pot = []
            for i in range(len(self.upper_coord)):
                point1 = self.upper_coord[i]
                point2 = self.lower_coord[i]
                x = self.calculate_val_x(point1, point2, y1)
                x_pot.append(x)
            
            for p in range(len(x_pot)-1):
                if x1 > float(x_pot[p]) and x1 < float(x_pot[p + 1]):
                    h = p
            if x1 < min(x_pot) or x1 > max(x_pot):
                # print("Coordinate outside the range of x")
                return 0
            elif x1 in x_pot:
                # print("Chess piece is placed on the edge of the squares")
                return 0
            else:
                # print('Chess piece position is:', pos[7-k,h])
                return pos[7-k,h]

def dict_to_board(chess_dict):
    # Initialize an 8x8 numpy array with empty strings
    chess_board = np.full((8, 8), "-", dtype=str)

    # Loop through the dictionary to populate the array
    for square, color in chess_dict.items():
        # Convert the square in chess notation to row and column in the numpy array
        col = ord(square[0]) - 97  # 'a' to 0, 'b' to 1, ..., 'h' to 7
        row = 8 - int(square[1])  # '1' to 7, '2' to 6, ..., '8' to 0

        # Convert the descriptive color to its shorthand ('w', 'b', or '-')
        if color == "white":
            value = 'w'
        elif color == "black":
            value = 'b'
        else:
            value = '-'

        # Place the shorthand color value in the corresponding position on the board
        chess_board[row, col] = value

    return chess_board

def board_to_dict(chess_board):
    # Create an empty dictionary to hold the square-color mappings
    chess_dict = {}
    
    # Rows correspond to the rank on the chess board (8 to 1)
    for row in range(8):
        # Columns correspond to the file on the chess board (a to h)
        for col in range(8):
            # Convert row and column to chess notation
            square = f"{chr(97 + col)}{8 - row}"
            
            # Get the value from the numpy array
            value = chess_board[row, col]
            
            # Convert the value to a descriptive word ("white", "black", or "empty")
            if value == 'w':
                color = "white"
            elif value == 'b':
                color = "black"
            else:
                color = "empty"
            
            # Add the square and its color to the dictionary
            chess_dict[square] = color

    return chess_dict

def findVertices(json_data,range_ymax = 15,range_ymin =5 ,range_xmax = 100, range_xmin=100):
    
    data = json_data

    # Extract points from data
    points = data["predictions"][0]["points"]
    
    first_edges = []
    sec_edges = []
    
    ##-----------------FIRST PART ( Y range )---------------------------------------------

    # Find the point with the minimum y value
    min_point = min(points, key=lambda point: point["y"])

    # Find the point with the maximum y value
    max_point = max(points, key=lambda point: point["y"])
    
    # Define the y value ranges
    y_max_range = (max_point["y"], max_point["y"] - range_ymax)
    y_min_range = (min_point["y"], min_point["y"] + range_ymin)

    # Filter points within the specified y value ranges
    points_within_y_max_range = [point for point in points if y_max_range[1] <= point["y"] <= y_max_range[0]]
    points_within_y_min_range = [point for point in points if y_min_range[0] <= point["y"] <= y_min_range[1]]

    # Sort the filtered points based on x-coordinate
    sorted_points_within_y_max_range = sorted(points_within_y_max_range, key=lambda point: point["x"])
    sorted_points_within_y_min_range = sorted(points_within_y_min_range, key=lambda point: point["x"])

    # Extract the required points
    min_x_within_y_max = sorted_points_within_y_max_range[0]
    max_x_within_y_max = sorted_points_within_y_max_range[-1]
    min_x_within_y_min = sorted_points_within_y_min_range[0]
    max_x_within_y_min = sorted_points_within_y_min_range[-1]

    # Create the 'edges' array by concatenating the points
    first_edges = [min_x_within_y_max, max_x_within_y_max, max_x_within_y_min, min_x_within_y_min]
    
    ## ----------------- SECOND PART (X range) -------------------------------

    # Find the point with the minimum x value
    min_point = min(points, key=lambda point: point["x"])

    # Find the point with the maximum x value
    max_point = max(points, key=lambda point: point["x"])
    
    print("min_point",min_point["x"])
    print("max_point",max_point["x"])
    
    # Find the mid point 
    mid_point = (min_point["x"]+max_point["x"])/2 
    print("mid_point",mid_point)

    # Define the x value ranges
    
    ## METHOD 1
    #  x_max_range = (max_point["x"], max_point["x"] - range_xmax)
    #  x_min_range = (min_point["x"], min_point["x"] + range_xmin)

    ## METHOD 2
    #  x_max_range = (max_point["x"],  (mid_point+range_xmax))
    #  x_min_range = (min_point["x"],  (mid_point-range_xmin))

    ## METHOD 3
    x_max_range = (max_point["x"],  (max_point["x"] - (max_point["x"]-mid_point)*0.5))
    x_min_range = (min_point["x"],  (min_point["x"] + (mid_point-min_point["x"])*0.5))
    

    # Filter points within the specified x value ranges
    points_within_x_max_range = [point for point in points if x_max_range[1] <= point["x"] <= x_max_range[0]]
    points_within_x_min_range = [point for point in points if x_min_range[0] <= point["x"] <= x_min_range[1]]

    # Sort the filtered points based on y-coordinate
    sorted_points_within_x_max_range = sorted(points_within_x_max_range, key=lambda point: point["y"])
    sorted_points_within_x_min_range = sorted(points_within_x_min_range, key=lambda point: point["y"])

    min_y_within_x_max = sorted_points_within_x_max_range[0]
    max_y_within_x_max = sorted_points_within_x_max_range[-1]
    min_y_within_x_min = sorted_points_within_x_min_range[0]
    max_y_within_x_min = sorted_points_within_x_min_range[-1]


    sec_edges = [max_y_within_x_min] + [max_y_within_x_max] + [min_y_within_x_max] + [min_y_within_x_min]

    
    ## ----------THIRD PART---------------------

    ### LEFT AND RIGHT LINE
    # Extract the first two points and the last two points of sec_edges
    first_two_points = [first_edges[0] , first_edges[3]]
    last_two_points = [first_edges[1] , first_edges[2]]
    

    # Extract x and y coordinates for the first and last two points
    x1_first, y1_first = zip(*[(point['x'], point['y']) for point in first_two_points])
    x2_first, y2_first = zip(*[(point['x'], point['y']) for point in last_two_points])

    ### TOP AND BOTTOM LINE
    # Extract the first two points and the last two points of sec_edges
    first_two_points = sec_edges[:2]
    last_two_points = sec_edges[2:]
    
    # Extract x and y coordinates for the first and last two points
    x1_sec, y1_sec = zip(*[(point['x'], point['y']) for point in first_two_points])
    x2_sec, y2_sec = zip(*[(point['x'], point['y']) for point in last_two_points])

    ### FIND INTERSECTION POINTS
    intersect_pts = []

    # compile lines 
    lines = [[x1_first, y1_first],[x1_sec, y1_sec],[x2_first, y2_first],[x2_sec, y2_sec],[x1_first, y1_first]]

    # Find the intersection point
    # Define the equations of the lines: y = m1 * x + b1, and y = m2 * x + b2
    for i in range(0,4):
        m1, b1 = np.polyfit(lines[i][0], lines[i][1], 1)  # Line 1
        m2, b2 = np.polyfit(lines[i+1][0], lines[i+1][1], 1)  # Line 2

        # Calculate the intersection point
        x_intersection = (b2 - b1) / (m1 - m2)
        y_intersection = m1 * x_intersection + b1

        #print(intersect_pts)
        intersect_pts += [[x_intersection,y_intersection]]

    
    return intersect_pts

def main(args=None):
    rclpy.init(args=args)
    
    turn_listener = TurnListener()  # Instantiate the listener for the turn
    chessboard_publisher = ChessBoardPublisher()  # Instantiate your chessboard publisher

    start_time = time.time()
    side_view = True
    chesspiece_iteration = 0
    cam_dict = {}

    # Initialize board position
    # chess_board = np.array([
    # ["b", "b", "b", "b", "b", "b", "b", "b"],
    # ["b", "b", "b", "b", "b", "b", "b", "b"],
    # ["-", "-", "-", "-", "-", "-", "-", "-"],
    # ["-", "-", "-", "-", "-", "-", "-", "-"],
    # ["-", "-", "-", "-", "-", "-", "-", "-"],
    # ["-", "-", "-", "-", "-", "-", "-", "-"],
    # ["w", "w", "w", "w", "w", "w", "w", "w"],
    # ["w", "w", "w", "w", "w", "w", "w", "w"]
    # ])

    chess_board = np.array([
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["-", "-", "-", "-", "-", "-", "-", "-"],
    ["w", "w", "w", "w", "w", "w", "w", "w"],
    ["w", "w", "w", "w", "w", "w", "w", "w"]
    ])
    chess_dict = board_to_dict(chess_board)

    # Load the Board segmentation model
    rf = Roboflow(api_key="NYOkOMoPHPLUgPpjUxvf")
    project = rf.workspace().project("chessboard-segmentation")
    model = project.version(1).model
    chessboard_mapped = 0
    print("Done loading project of chessboard weights")

    # Load the chess piece detection model
    project2 = rf.workspace().project("chess-pieces-2-etbrl")
    model2 = project2.version(1).model
    print("Done loading project of chess-piece weights")
    
    cap = cv.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    # As long as rclpy is not shutdown, continue running the loop
    while rclpy.ok():
        rclpy.spin_once(turn_listener, timeout_sec=0)  # Non-blocking
        print("turn_listener.white_is_playing: ", turn_listener.white_is_playing)
        if turn_listener.white_is_playing:
            # Capture frame-by-frame
            ret, frame = cap.read()

            # If frame is read correctly, ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            # Image Processing
            # Map chessboard -- only once
            if not chessboard_mapped:

                cv.imwrite(chessboard_path, frame)
                json_data = model.predict(chessboard_path).json()
                # json_str = json.dumps(json_data, indent=4) # pretty json_data
                # print (json_str)

                im = cv.imread(chessboard_path)
                # cv.imshow("chessboard", im)  
                # implot = plt.imshow(im)
                # plt.imshow(im)
                
                if len(json_data['predictions']) > 0:
                    print ("Chessboard detected")
                    segment_res = np.array(json_data['predictions'][0]['points'])
                    
                    # Display points
                    for point in segment_res:
                        # plt.scatter(point['x'], point['y'], color="red", s=10)
                        im_chessboard = cv.circle(im, (int(point['x']), int(point['y'])), 2, (0,0,255), -1)
                    # plt.show()
                    # cv.imshow("chessboard boundary", im_chessboard)  

                    chessboard_mapped = 1
                    print ("Chessboard segmented")
                else:
                    print ("Chessboard not detected")

                # Process the chessboard
                print ("Chessboard corner processing...")

                if side_view:
                    corners = findVertices(json_data)
                else:
                    corners = findVertices(json_data)
                    corners[1][0] = corners[1][0] + 7 # offset due to model not properly segmented the bottom right
                    corners[2][1] = corners[2][1] + 2 # offset due to model not properly segmented the top right
                print("corners:", corners)
                for point in corners:
                    # plt.scatter(point['x'], point['y'], color="red", s=10)
                    im_chessboard_corners = cv.circle(im, (int(point[0]), int(point[1])), 10, (0,255,255), -1)
                # plt.show()
                cv.imshow("chessboard corners", im_chessboard_corners)
                print ("Chessboard corners plotted")

                # Each boxes
                trapezium_x = [corners[i][0] for i in [3, 2, 1, 0, 3]]
                
                min_y = (corners[2][1] + corners[3][1]) / 2
                max_y = (corners[0][1] + corners[1][1]) / 2

                k_side_view = [0.5/6, 0.5/6, 0.65/6, 0.7/6, 0.7/6, 0.85/6, 1/6, 1.1/6]
                k_top_view = [0.65/6.3, 0.7/6.3, 0.75/6.3, 0.8/6.3, 0.85/6.3, 0.85/6.3, 0.9/6.3, 0.9/6.3]

                y_coord = [max_y]
                
                if side_view:
                    for i in range(7, -1, -1 ):  # 8 more y-values to make a total of 9
                        y_coord.append(y_coord[-1] - (max_y - min_y) * k_side_view[i])
                else:
                    # y_coord = np.linspace((corners[2][1]+corners[3][1])/2, (corners[0][1]+corners[1][1])/2, 9)
                    for i in range(7, -1, -1 ):  # 8 more y-values to make a total of 9
                        y_coord.append(y_coord[-1] - (max_y - min_y) * k_top_view[i])
                
                trapezium = Trapezium(trapezium_x, y_coord, chessboard_path)
                trapezium.plot_graph()
                
            # Map Chess Piece
            if chessboard_mapped and (time.time() - start_time > chesspiece_delay) and chesspiece_iteration < 5:
                chesspiece_iteration += 1
                print(f"{chesspiece_iteration}th chesspiece iteration")
                print("Mapping chesspiece")
                start_time = time.time()
                cv.imwrite(chesspiece_path, frame)
                json_data2 = model2.predict(chesspiece_path).json()
                # json_str2 = json.dumps(json_data2, indent=4) # pretty json_data2
                # print (json_str2)
                im_chesspiece = cv.imread(chesspiece_path)
                im_chesspiece_on_chessboard = cv.imread(chessboard_mapped_path)

                # Loop through all predictions
                for prediction in json_data2['predictions']:
                    x = int(prediction['x'])
                    y = int(prediction['y'])
                    height = int(prediction['height'])
                    chess_class = prediction['class']
                    color = (0, 255, 0)  # Default to green for any unknown class
                    
                    # Determine the color of the circle based on the chess piece color
                    if 'white' in chess_class:
                        chess_class = 'white'
                        color = (255, 255, 255)  # White for white pieces
                    elif 'black' in chess_class:
                        chess_class = 'black'
                        color = (119,136,153)  # Black (LightSlateGray) for black pieces
                        continue

                    # Draw a circle around the chess piece
                    # Syntax: cv.circle(image, center_coordinates, radius, color, thickness)
                    if side_view:
                        im_chesspiece = cv.circle(im_chesspiece, (x, y+int(height/4)), 10, color, -1)
                        im_chesspiece_on_chessboard = cv.circle(im_chesspiece_on_chessboard, (x, y+int(height/4)), 10, color, -1)
                        temp = f'{trapezium.find_pos(x, y+int(height/2), side_view)}'
                    else:
                        im_chesspiece = cv.circle(im_chesspiece, (x, y), 10, color, -1)
                        im_chesspiece_on_chessboard = cv.circle(im_chesspiece_on_chessboard, (x, y), 10, color, -1)
                        temp = f'{trapezium.find_pos(x, y, side_view)}'
                    
                    if temp != '0' and chess_class != 'black':
                        cam_dict[temp]=chess_class
                
                cv.imshow("Chesspiece", im_chesspiece)
                cv.imshow("Chesspiece2", im_chesspiece_on_chessboard)
                

                # compare cam_dict with chess_dict
                if chesspiece_iteration == 5:
                    print("cam_dict: \n", dict_to_board(cam_dict))
                    print("chess_board: \n", chess_board)
                    # print("chess_dict: \n", json.dumps(chess_dict, indent=4))
                    for key, value in cam_dict.items():
                        if key in chess_dict:
                            if chess_dict[key] != value:
                                # opponent_move = False
                                
                                ## METHOD 1
                                # # Capture a frame and subtract to see a difference
                                # for i in range (5):
                                #     ret, frame = cap.read()
                                #     im = cv.imread(chessboard_path)
                                #     cv.imshow("difference", frame-im)
                                #     cv.imwrite("difference.jpg", frame-im)
                                #     # if (detect_movement(frame, im)):
                                #     #     print("Opponent movement detected")
                                
                                ## METHOD 2
                                # Compare the chess in chess_dict with cam_dict
                                for key2, value2 in chess_dict.items():
                                    if 'white' in value2 and key2 not in cam_dict:
                                            chess_dict[key2] = 'empty'
                                            chess_dict[key] = value
                                            break
                                
                                chess_board = dict_to_board(chess_dict)
                                print(chess_board)
                                break
                            
                    chesspiece_iteration = 0
                    cam_dict = {}
                    chessboard_publisher.publish_board(chess_board)


            # Display the resulting frame
            cv.imshow('frame', frame)
            
            if cv.waitKey(1) == ord('q'):
                break

        else:
            # Black turn, do nothing
            # Sleep for a short duration to reduce CPU usage.
            time.sleep(0.1)

        # Check for shutdown signal (Ctrl+C) and handle other GUI events
        if cv.waitKey(1) == ord('q'):
            break

    # When everything done, release the capture and destroy ROS nodes
    cap.release()
    cv.destroyAllWindows()
    chessboard_publisher.destroy_node()
    turn_listener.destroy_node()
    rclpy.shutdown()

# Run the main function
if __name__ == "__main__":
    main()