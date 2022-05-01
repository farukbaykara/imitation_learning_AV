import cv2
import numpy as np
from sim_test import *


cap = cv2.VideoCapture("test.mp4")

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
print(frame_width)
print(frame_height)

# out = cv2.VideoWriter('output_sim.mp4', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (1280, 720), 3)
size = (1280, 720)
fourcc = cv2.VideoWriter_fourcc('D', 'I', 'V', 'X')
out = cv2.VideoWriter('out_sim.mp4', fourcc, 15, size)

if (cap.isOpened()== False): 
  print("Error opening video stream or file")

cnt = 0
while cap.isOpened() and cnt < 1000:
    ret, frame = cap.read()
    if ret:

        resized_img = cv2.resize(frame, (1280, 720))
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 0)
        left_lane_points, right_lane_points = get_lane_points(blurred_image)
        print(cnt)

        img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)

        for i in range(len(left_lane_points)):
            cv2.circle(resized_img, left_lane_points[i], 5, (210, 23, 200), 3)
            cv2.circle(resized_img, right_lane_points[i], 5, (210, 178, 23), 3)

        # print resized_img.shape
        out.write(resized_img)

    else:
        break
    print(cnt)
    cnt += 1

print("Writing")

cap.release()
out.release()
