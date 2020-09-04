import cv2
import numpy as np
import glob

# reference: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
def extract_mask(frame):
    temp = frame.copy()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    # low = np.array([48,101,191])
    low = np.array([48,156,222])
    high = np.array([150,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, low, high)
    kernel = np.ones((7,7),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Bitwise-AND mask and original image
    return mask

def reverse(frame):
    temp = frame.copy()
    b = temp[:,:,0].astype('int')
    g = temp[:,:,1].astype('int')
    r = temp[:,:,2].astype('int')
    test = 50+r
    indices = g > test
    foo = np.array(r>100)
    indices1 = np.logical_and(indices, foo)
    # make green the avg of red and blue where green is greater than red+50
    g[indices] = r[indices] + b[indices]
    
    g[indices] /= 2

    # on the averaged pixels, if red is high, then set green to red (this makes these pixels yellow)
    g[indices1] = r[indices1]
    # plug newly formatted green channel back in
    temp[:,:,1] = g
    return temp

def main():

    data_dir = "/home/subbu/catkin_ws/src/autovalet/autovalet_lane_detection/data/"
    cur_data = data_dir + "green_left/*.jpg"
    label_dir = "/home/subbu/catkin_ws/src/autovalet/autovalet_lane_detection/data/training_label/"
    image_dir = "/home/subbu/catkin_ws/src/autovalet/autovalet_lane_detection/data/training_data/"

    for img in sorted(glob.glob(cur_data)):
        cv_img = cv2.imread(img)
        mask = extract_mask(cv_img)
        ip_img = reverse(cv_img)
        cv2.imshow('mask', mask)
        cv2.imshow('input', ip_img)
        cv2.waitKey(0)
        # print(label_dir+"image"+img[-9:-4])
        # new_number = string(int(img[-9:-4])+1000)
        # print(label_dir+"image"+ img[-9:])

        # cv2.imwrite(label_dir+"image"+ img[-9:], mask)
        # cv2.imwrite(image_dir+"image"+ img[-9:], ip_img)

if __name__ == '__main__':
	main()