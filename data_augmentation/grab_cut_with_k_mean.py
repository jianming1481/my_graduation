#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import cv2
import sys

if __name__ == '__main__':

    # Loading images
    if len(sys.argv) > 2:
	# The input label image
        filename = sys.argv[1]
	# The input rgb image
        filename2 = sys.argv[2]

    else:
        print("No input image given, so loading default image, ../data/lena.jpg \n")
        print("Correct Usage: python grabcut.py <ground truth filename> <original image filename> \n")
        filename = '/home/iclab-giga/Documents/trainning_data_gt_00014.jpg'
        filename2 = '/home/iclab-giga/Documents/trainning_data_00014.jpg'

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

    # Declare for grabcut algorithm
    bgdModel = np.zeros((1,65),np.float64)
    fgdModel = np.zeros((1,65),np.float64)
    mask = np.zeros(img2.shape[:2], np.uint8)

    cv2.grabCut(img2, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
    mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    output = np.zeros(img2.shape, np.uint8)  # output image to be shown
    output = cv2.bitwise_and(img2, img2, mask=mask2)
    
    # Use K-mean to segment the remaining background
    Z = output.reshape((-1,3))

    # convert to np.float32
    Z = np.float32(Z)

    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 2
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)

    center = np.uint8(center)
    for i in range(center.shape[0]):
        if center[i][0] < 60 and center[i][1] < 60 and center[i][2] < 60:
            center[i] = [0, 0, 0]

    # Now convert back into uint8, and make original image
    center = np.uint8(center)
    res = center[label.flatten()]
    output = res.reshape((output.shape))

    cv2.imshow('test',output)
    while(1):
        k = cv2.waitKey(60)
        if k == 27:  # Esc key to stop
            break

    # cv2.rectangle(output, (min_x, min_y), (max_x, max_y),(0,255,0),3)
    # plt.imshow(output),plt.colorbar(),plt.show()

    #cv2.imshow('res',img_bg)
    #while(1):
    #    k = cv2.waitKey(60)
    #    if k == 27:  # Esc key to stop
    #        break
