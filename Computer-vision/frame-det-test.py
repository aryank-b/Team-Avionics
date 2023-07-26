import cv2
import sys
import numpy as np

delay = 1

cap = cv2.VideoCapture('/dev/v4l/by-id/usb-SunplusIT_Inc_HP_TrueVision_HD_Camera-video-index0')
if not cap.isOpened():
        sys.exit()

def aspect_ratio(cnt):
        x,y,w,h = cv2.boundingRect(cnt)
        ar = float(w)/float(h)
        return ar
def center(cnt):
        M = cv2.moments(cnt)
        if(M['m00']!=0):
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                return cx,cy
        else:
                return 0,0
def mask_red(frame):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
         
        # lower boundary RED color range values; Hue (0 - 10)
        red_lower1 = np.array([0, 160, 20])
        red_upper1 = np.array([8, 255, 255])
        
        # upper boundary RED color range values; Hue (160 - 180)
        red_lower2 = np.array([160,160,20])
        red_upper2 = np.array([179,255,255])
        
        red_lower_mask = cv2.inRange(hsv, red_lower1, red_upper1)
        red_upper_mask = cv2.inRange(hsv, red_lower2, red_upper2)
        
        red_mask = red_lower_mask + red_upper_mask
        return red_mask

def mask_green(frame, ox, oy):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width, _ = frame.shape
        hsv[oy:, 0:width] = 0
        green_lower = np.array([40, 100, 20])
        green_upper = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        return green_mask

while True:
    ret, frame = cap.read()
    if ret:
        height, width, _ = frame.shape
        cx = int(width/2)
        cy = int(height/2)

        red_mask = mask_red(frame)
        red_contours, red_hierarchy = cv2.findContours(image=red_mask, mode=cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)
        red_c = max(red_contours, key=cv2.contourArea)
        ar = aspect_ratio(red_c)
        ox, oy = center(red_c)
        image_copy = frame.copy()
        if(ar<1.05):
                rect = cv2.minAreaRect(red_c)
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.drawContours(image=image_copy, contours=[box], contourIdx=0, color=(0,255,0), thickness=2, lineType=cv2.LINE_AA)
                image_copy = cv2.circle(image_copy, (ox, oy), 5, (25, 25, 25), 3)
                green_mask = mask_green(frame, ox, oy)
                green_contours, green_hierarchy = cv2.findContours(image=green_mask, mode=cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)
                if(green_contours):
                        green_c = max(green_contours, key=cv2.contourArea)
                        gx, gy = center(green_c)
                        if(gx!=0 and gy!=0):
                                image_copy = cv2.circle(image_copy, (gx, gy), 3, (25, 25, 25), 3)
                        cv2.imshow('green mask', green_mask)

        cv2.imshow('frame copy',image_copy)
        cv2.imshow('red mask', red_mask)
    if (cv2.waitKey(delay) & 0xFF == ord('q')):
        break

cv2.destroyAllWindows()