import os
import cv2
import numpy as np

def nothing(x):
    pass
# Creating a window for later use
cv2.namedWindow('result', cv2.WINDOW_NORMAL)

# Starting with 100's to prevent error while masking
h1,l1,s1 = 0,0,0
h2,l2,s2 = 180,255,255

# Creating track bar
cv2.createTrackbar('h1', 'result',0,180,nothing)
cv2.createTrackbar('l1', 'result',0,255,nothing)
cv2.createTrackbar('s1', 'result',0,255,nothing)
cv2.createTrackbar('h2', 'result',0,180,nothing)
cv2.createTrackbar('l2', 'result',0,255,nothing)
cv2.createTrackbar('s2', 'result',0,255,nothing)

while(1):

    frame = cv2.imread(os.environ.get('HOME') + '/Pictures/simturn.png')

    #converting to HSV
    hls = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)

    # get info from track bar and appy to result
    h1 = cv2.getTrackbarPos('h1','result')
    l1 = cv2.getTrackbarPos('l1','result')
    s1 = cv2.getTrackbarPos('s1','result')
    h2 = cv2.getTrackbarPos('h2','result')
    l2 = cv2.getTrackbarPos('l2','result')
    s2 = cv2.getTrackbarPos('s2','result')

    # Normal masking algorithm
    #lower_yellow = np.array([15,51, 230])
    lower_yellow = np.array([h1,l1,s1])
    upper_yellow = np.array([h2,l2,s2])

    mask = cv2.inRange(hls,lower_yellow, upper_yellow)

    result = cv2.bitwise_and(frame,frame,mask = mask)

    cv2.imshow('result',result)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
