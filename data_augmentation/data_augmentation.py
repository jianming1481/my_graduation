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

    # Generate the List for Neural Netwrok to Load Data 
    global f
    # f = open('/home/iclab-giga/graduate_ws/src/data_augmentation/data/train.txt','w')
    f = open('/home/iclab-giga/Documents/TEST_DATA_AUG/train.txt','w')
    # g = open('/home/iclab-giga/graduate_ws/src/data_augmentation/data/val.txt','w')
    g = open('/home/iclab-giga/Documents/TEST_DATA_AUG/val.txt','w')
    g.close()

    # Read Image from List to Augment Data
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
    # for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/ori_img/*.jpg')):
    for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/ori_img/*.jpg')):
        filename_ori_list.append(ori_filename)
    # for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/label_img/*.jpg')):
    for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/label_img/*.jpg')):
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
    output = cv2.bitwise_and(img, img, mask=    mask2)
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
    # g = open('/home/iclab-giga/graduate_ws/src/data_augmentation/data/val.txt', 'a')
    g = open('/home/iclab-giga/Documents/TEST_DATA_AUG/val.txt','w')

    """
        Save the original RGB Image and Label Image as Training Data
    """
    file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
    cv2.imwrite(file_name,ori_img)
    f.write(file_name+" ")
    file_name = 'data/label_img/training_data_'+str(foto_index)+'.jpg'
    label_img_8UC1 = cv2.cvtColor(label_img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(file_name,label_img_8UC1)
    f.write(file_name+"\n")
    foto_index += 1

    """
        Use the Original Data to Augment Data
        Using IR_Img(Depth) to Find the ROI for Grab Cut Algorithm
    """
    min_x, min_y, max_x, max_y = find_ROI(label_img)
    rect = (min_x,min_y,abs(max_x-min_x),abs(max_y-min_y))
    ori_item = my_grab_cut(ori_img,rect)
    label_item = my_grab_cut(label_img,rect)

    """
        After Grab Cut, There Are Some Shadow Remain
        It will Affect me to generate the ground truth data.
        So I use K-means algorithm and color histogram to split it out from Image
        Using Methodology method to refine the label Image in the end
    """
    # Init params for K-means
    K = 8
    Z = ori_item.reshape((-1,3))
    # convert to np.float32
    Z = np.float32(Z)
    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)

    # To find the background's label 
    # In my case black color mean background
    # B,G,R
    center = np.uint8(center)
    for i in range(center.shape[0]):
        if center[i][0] < 60:
            if center[i][1] < 60:
                if center[i][2] < 60:
                    center[i] = [0, 0, 0]
    # Now convert back into uint8, and make original RGB image
    res = center[label.flatten()]
    res2 = res.reshape((ori_item.shape))

    # Using Methodology Method to Refine the Label Img
    kernel = np.ones((5,5),np.uint8)
    res2 = cv2.dilate(res2,kernel,iterations = 4)
    res2 = cv2.erode(res2,kernel,iterations = 4)
    res2 = cv2.cvtColor(res2,cv2.COLOR_BGR2GRAY)
    """
        Now we have the label Img
        Using Label Image as a Mask to grab the item from original RGB-IMG (After Grab Cut)
        And block black thing
    """
    ori_item = cv2.bitwise_and(ori_img,ori_img,mask = res2)

    # Transfer ori_item to label_item
    label_item = cv2.cvtColor(ori_item,cv2.COLOR_BGR2GRAY)
    ret, label_item = cv2.threshold(label_item, 10, 1, cv2.THRESH_BINARY)

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
            # file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
            file_name = '/home/iclab-giga/Documents/TEST_DATA_AUG/gen/ori_img/training_data_'+str(foto_index)+'.jpg'
            cv2.imwrite(file_name,tmp_ori_img)
            g.write(file_name+" ")
            # file_name = 'data/label_img/training_data_'+str(foto_index)+'.jpg'
            file_name = '/home/iclab-giga/Documents/TEST_DATA_AUG/gen/label_img/training_data_'+str(foto_index)+'.jpg'
            # tmp_label_img_8UC1 = cv2.cvtColor(tmp_label_img, cv2.COLOR_BGR2GRAY)
            tmp_label_img_8UC1 = tmp_label_img
            cv2.imwrite(file_name,tmp_label_img_8UC1)
            g.write(file_name+"\n")
        else:
            # file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
            file_name = '/home/iclab-giga/Documents/TEST_DATA_AUG/gen/ori_img/training_data_'+str(foto_index)+'.jpg'
            cv2.imwrite(file_name,tmp_ori_img)
            f.write(file_name+" ")
            # file_name = 'data/label_img/training_data_'+str(foto_index)+'.jpg'
            file_name = '/home/iclab-giga/Documents/TEST_DATA_AUG/gen/label_img/training_data_'+str(foto_index)+'.jpg'
            # tmp_label_img_8UC1 = cv2.cvtColor(tmp_label_img, cv2.COLOR_BGR2GRAY)
            tmp_label_img_8UC1 = tmp_label_img
            cv2.imwrite(file_name,tmp_label_img_8UC1)
            f.write(file_name+"\n")
        foto_index += 1
    g.close()

if __name__ == "__main__":
    global foto_index
    foto_index = 1
    main()