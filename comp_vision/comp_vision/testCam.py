import numpy as np
import cv2 as cv
import json
from roboflow import Roboflow

# Load the Board segementation model
rf = Roboflow(api_key="NYOkOMoPHPLUgPpjUxvf")
project = rf.workspace().project("chessboard-segmentation")
model = project.version(1).model
chessboard_mapped = 0

#Load the chess piece detection model
# project2 = rf.workspace().project("chess-pieces-2-etbrl")
# model2 = project2.version(1).model

cap = cv.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Image Processing
    if not(chessboard_mapped):
        cv.imwrite('first.jpg', frame)

    # Display the resulting frame
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()