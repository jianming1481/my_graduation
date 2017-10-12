#!/usr/bin/python
#__author__ = "Chien-Ming Lin"
#__copyright__ = "Copyright 2017, The Semantic Segmentation Project"
#__credits__ = "Chien-Ming Lin"
#__license__ = "GPL"
#__version__ = "1.0.0"
#__maintainer__ = "Chien-Ming Lin"
#__email__ = "jianming1481@gmail.com"

# Import Libraries
import cv2
import random
import glob
import numpy as np
import sys
from matplotlib import pyplot as plt


# Declare Global Variable
f = None
global foto_index

def main():
    # Load the Path of data into a list
    filename_ori_list, filename_label_list = load_data()

    # Augment the data and Generate the text document for loading data
    global f
    f = open('/home/iclab-giga/graduate_ws/src/data_augmentation/data/train.txt','w')
    g = open('/home/iclab-giga/graduate_ws/src/data_augmentation/data/val.txt','w')
    g.close()
    for i in range(1,len(filename_ori_list)+1):
        m_str = filename_ori_list.pop()
        ori_img = cv2.imread(m_str)
        m_str = filename_label_list.pop()
        label_img = cv2.imread(m_str)
        aug_data(ori_img, label_img)
    print("==================Finish data augmentation==================\n")
    f.close()

def load_data():
    # Declare Empty list to store the path of data
    filename_ori_list = []
    filename_label_list = []

    # Search all the jpg file in directories
    for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/ori_img/*.jpg')):
        filename_ori_list.append(ori_filename)
    for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/label_img/*.jpg')):
        filename_label_list.append(label_filename)

    print "The number of image need to process: ", len(filename_ori_list)
    print len(filename_ori_list) * 20, "images will be generated"
    filename_ori_list.reverse()
    filename_label_list.reverse()
    return filename_ori_list, filename_label_list

# To do grab cut algorithm, we use the labeled image to find ROI for futher work
def find_ROI(label_img):
    min_x = 999
    min_y = 999
    max_x = 0
    max_y = 0

    # Transfer to 1 channel cuz I only need one value to determine whether the pixel is labeled or no
    img_8UC1 = cv2.cvtColor(label_img, cv2.COLOR_BGR2GRAY)

    for y in range(0,480):
        for x in range(0,640):
            if img_8UC1[y,x]>0:
                if x < min_x:
                    min_x = x
                if y < min_y:
                    min_y = y
                if x > max_x:
                    max_x = x
                if y > max_y:
                    max_y = y
    return (min_x,min_y,max_x,max_y)

def my_grab_cut(img, rect):
    # Declare for algorithm
    bgdModel = np.zeros((1, 65), np.float64)
    fgdModel = np.zeros((1, 65), np.float64)
    mask = np.zeros(img.shape[:2], np.uint8)

    cv2.grabCut(img, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
    mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
    output = np.zeros(img.shape, np.uint8)  # output image to be shown
    output = cv2.bitwise_and(img, img, mask=mask2)
    # cv2.imshow('test',output)
    # while(1):
    #     k = cv2.waitKey(60)
    #     if k == 27:  # Esc key to stop
    #         break
    return output

def add_obj_with_bg(tmp_ori_img):
    # Load back ground image
    img_bg = cv2.imread('/home/iclab-giga/graduate_ws/back_ground.jpg')
    # I want to put logo on top-left corner, So I create a ROI
    rows, cols, channels = tmp_ori_img.shape
    roi = img_bg[0:rows, 0:cols]

    # Now create a mask of logo and create its inverse mask also
    outputgray = cv2.cvtColor(tmp_ori_img, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(outputgray, 10, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)
    # Now black-out the area of logo in ROI
    img1_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)
    # Take only region of logo from logo image.
    img2_fg = cv2.bitwise_and(tmp_ori_img, tmp_ori_img, mask=mask)

    # Put logo in ROI and modify the main image
    dst = cv2.add(img1_bg, img2_fg)
    img_bg[0:rows, 0:cols] = dst
    return img_bg
    # cv2.imshow('res',img_bg)
    # while(1):
    #     k = cv2.waitKey(60)
    #     if k == 27:  # Esc key to stop
    #         break

def aug_data(ori_img, label_img):
    # Declare Global Variable
    global foto_index
    global f
    g = open('/home/iclab-giga/graduate_ws/src/data_augmentation/data/val.txt', 'a')

    # Save the original image to data

    file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
    cv2.imwrite(file_name,ori_img)
    f.write(file_name+" ")
    file_name = 'data/label_img/training_data_'+str(foto_index)+'.jpg'
    label_img_8UC1 = cv2.cvtColor(label_img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(file_name,label_img_8UC1)
    f.write(file_name+"\n")
    foto_index += 1

    # Doing grab cut for the data
    min_x, min_y, max_x, max_y = find_ROI(label_img)
    rect = (min_x,min_y,abs(max_x-min_x),abs(max_y-min_y))
    ori_item = my_grab_cut(ori_img,rect)
    label_item = my_grab_cut(label_img,rect)

    for i in range(2,21):
        # Init some parameter to change scale or rotate or translate
        max_rows, max_cols = ori_img.shape[:2]
        random.seed(None)
        scale = random.random()
        rot = random.randint(-180,180)
        rows = random.randint(max_rows/3,max_rows)
        cols = random.randint(max_cols/3,max_cols)
        
        # Make some limit for parameter
        while (scale < 0.3):
            scale = random.random()
            
        # Build Rotate Matrix
        M = cv2.getRotationMatrix2D((cols/2,rows/2),rot,scale)
        
        # Rotate Image
        tmp_ori_img = cv2.warpAffine(ori_item,M,(max_cols,max_rows))
        tmp_label_img = cv2.warpAffine(label_item,M,(max_cols,max_rows))

        tmp_ori_img = add_obj_with_bg(tmp_ori_img)

        # Save Image
        if i>15:
            file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
            cv2.imwrite(file_name,tmp_ori_img)
            g.write(file_name+" ")
            file_name = 'data/label_img/training_data_'+str(foto_index)+'.jpg'
            tmp_label_img_8UC1 = cv2.cvtColor(tmp_label_img, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(file_name,tmp_label_img_8UC1)
            g.write(file_name+"\n")
        else:
            file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
            cv2.imwrite(file_name,tmp_ori_img)
            f.write(file_name+" ")
            file_name = 'data/label_img/training_data_'+str(foto_index)+'.jpg'
            tmp_label_img_8UC1 = cv2.cvtColor(tmp_label_img, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(file_name,tmp_label_img_8UC1)
            f.write(file_name+"\n")
        foto_index += 1
    g.close()

if __name__ == "__main__":
    global foto_index
    foto_index = 1
    main()