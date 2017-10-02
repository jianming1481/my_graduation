#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import cv2
import sys

if __name__ == '__main__':

    # Loading images
    if len(sys.argv) > 2:
        filename = sys.argv[1] # for drawing purposes
        filename2 = sys.argv[2] # for drawing purposes

    else:
        print("No input image given, so loading default image, ../data/lena.jpg \n")
        print("Correct Usage: python grabcut.py <filename> \n")
        filename = '../data/lena.jpg'

    min_x=999
    min_y=999
    max_x=1
    max_y=1
    img = cv2.imread(filename,cv2.CV_8UC1)
    img2 = cv2.imread(filename2)
    # cv2.imshow('test',img)
    # while(1):
    #     k = cv2.waitKey(60)
    #     if k == 27:  # Esc key to stop
    #         break

    for y in range(0,480):
        for x in range(0,640):
            if img[y,x]>0:
                if x < min_x:
                    min_x = x
                if y < min_y:
                    min_y = y
                if x > max_x:
                    max_x = x
                if y > max_y:
                    max_y = y
    print "[%d,%d,%d,%d]" %(max_x,max_y,min_x,min_y)
    rect = (min_x, min_y, abs(max_x-min_x), abs(max_y-min_y))

    # Declare for algorithm
    bgdModel = np.zeros((1,65),np.float64)
    fgdModel = np.zeros((1,65),np.float64)
    mask = np.zeros(img2.shape[:2], np.uint8)

    cv2.grabCut(img2, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
    mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    output = np.zeros(img2.shape, np.uint8)  # output image to be shown
    output = cv2.bitwise_and(img2, img2, mask=mask2)
    # cv2.imshow('test',output)
    # while(1):
    #     k = cv2.waitKey(60)
    #     if k == 27:  # Esc key to stop
    #         break
    # cv2.rectangle(output, (min_x, min_y), (max_x, max_y),(0,255,0),3)
    # plt.imshow(output),plt.colorbar(),plt.show()

    # Load back ground image
    img_bg = cv2.imread('Canon_sumidero.jpg')
    # I want to put logo on top-left corner, So I create a ROI
    rows, cols, channels = output.shape
    roi = img_bg[0:rows, 0:cols]

    # Now create a mask of logo and create its inverse mask also
    outputgray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(outputgray, 10, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)
    # Now black-out the area of logo in ROI
    img1_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)
    # Take only region of logo from logo image.
    img2_fg = cv2.bitwise_and(output, output, mask=mask)

    # Put logo in ROI and modify the main image
    dst = cv2.add(img1_bg, img2_fg)
    img_bg[0:rows, 0:cols] = dst

    cv2.imshow('res',img_bg)
    while(1):
        k = cv2.waitKey(60)
        if k == 27:  # Esc key to stop
            break