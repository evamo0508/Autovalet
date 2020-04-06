import cv2
import numpy as np
import glob

# reference: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
def extract_mask(frame):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([48,101,191])
    upper_blue = np.array([180,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    # for channel in range(3):
    frame[mask == 255] -= np.array([0,174,0], dtype='uint8')

    # cv2.imshow('frame',frame)
    # cv2.imshow('mask',mask)
    # cv2.imshow('res',res)
    # cv2.waitKey(0)
    return frame

def main():

    data_dir = "/home/subbu/catkin_ws/src/autovalet/lane_detection/data/"
    cur_data = data_dir + "green/*.jpg"
    for img in sorted(glob.glob(cur_data)):
        cv_img = cv2.imread(img)
        frame = extract_mask(cv_img)
        cv2.imshow('raw', cv_img)
        cv2.imshow('mod', frame)
        cv2.waitKey(50)

if __name__ == '__main__':
	main()