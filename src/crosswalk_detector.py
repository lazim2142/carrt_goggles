#!/usr/bin/python2.7
import numpy as np
import numpy.core.multiarray

import cv2

def to_hsv(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return hsv_img
    
def detect_white(img):
    hsv_img = to_hsv(img)
    sensitivity = 100
    lower_white = np.array([0,0,255-sensitivity], dtype=np.uint8)
    upper_white = np.array([255,sensitivity,255], dtype=np.uint8)

    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv_img, lower_white, upper_white)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(hsv_img,hsv_img, mask= mask)
         
    whiteCount = cv2.countNonZero(mask)
    return whiteCount
    
def detect_red(img):
    hsv_img = to_hsv(img)
    
    lower_red = np.array([0,50,50], dtype=np.uint8)
    upper_red = np.array([10,255,255], dtype=np.uint8)

    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv_img, lower_red, upper_red)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(hsv_img,hsv_img, mask= mask)
         
    redCount = cv2.countNonZero(mask)
    return redCount
    

def to_gray(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return gray_img   
	
    
def color_detect(img):

    whiteCount = detect_white(img)
    redCount = detect_red(img)
    
    if whiteCount > redCount: print("CROSS")
    elif redCount >= whiteCount: print("DO NOT CROSS")


if __name__ == '__main__':
    cv2.namedWindow("output", cv2.WINDOW_NORMAL)  
    cascade = cv2.CascadeClassifier('cascade.xml')
    #cap = cv2.VideoCapture("video1.mp4")

    while 1:
        #ret, img = cap.read()
        img = cv2.imread("image2.bmp")
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = to_gray(img)
        matches = cascade.detectMultiScale(gray, 1.3, 5)       
        lightFound = False
        for (x,y,w,h) in matches:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            #cropped = img[y: y + h, x: x + w]
            cropped = img[y + int(0.1*h): y + int(0.9*h), x + int(0.1*w): x + int(0.9*w)]
            #print("Match found!\n")
            lightFound = True

        cv2.imshow("output",img)
        #cv2.imshow("img",cropped)
        if lightFound == True:
            color_detect(cropped)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    #cap.release()
    cv2.destroyAllWindows()