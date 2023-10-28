import numpy as np
import cv2 as cv
import json
from roboflow import Roboflow
import matplotlib.pyplot as plt

chessboard_path = 'chessboard-ref.jpg'
chesspiece_path = 'chesspiece-ref.jpg'

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
#     x_max_range = (max_point["x"], max_point["x"] - range_xmax)
#     x_min_range = (min_point["x"], min_point["x"] + range_xmin)

    ## METHOD 2
#     x_max_range = (max_point["x"],  (mid_point+range_xmax))
#     x_min_range = (min_point["x"],  (mid_point-range_xmin))

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

def main():
    # Load the Board segmentation model
    rf = Roboflow(api_key="NYOkOMoPHPLUgPpjUxvf")
    project = rf.workspace().project("chessboard-segmentation")
    model = project.version(1).model
    chessboard_mapped = 0
    print("Done loading project of chessboard weights")

    # Load the chess piece detection model (commented out for now)
    project2 = rf.workspace().project("chess-pieces-2-etbrl")
    model2 = project2.version(1).model
    print("Done loading project of chess-piece weights")

    cap = cv.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    while True:
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
            corners = findVertices(json_data)
            print("corners:", corners)  
            for point in corners:
                # plt.scatter(point['x'], point['y'], color="red", s=10)
                im_chessboard_corners = cv.circle(im, (int(point[0]), int(point[1])), 10, (0,255,255), -1)
            # plt.show()
            cv.imshow("chessboard corners", im_chessboard_corners)
            print ("Chessboard corners plotted")

        

        # Display the resulting frame
        cv.imshow('frame', frame)
        
        if cv.waitKey(1) == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()

# Run the main function
if __name__ == "__main__":
    main()