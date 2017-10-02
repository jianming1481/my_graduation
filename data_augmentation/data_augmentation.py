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

# Declare Global Variable
ori_img = None
label_img = None
image_list = None
f = None
global foto_index

def main():
    # Declare Global Variable
    global ori_img
    global image_list
    filename_ori_list = []
    filename_label_list = []
    
    # Search all the jpg file in directories
    for ori_filename in sorted(glob.glob('/home/iclab-gtx1080/Documents/training_Data/my_data/ori_img/*.jpg')):
        filename_ori_list.append(ori_filename) 
        
    for label_filename in sorted(glob.glob('/home/iclab-gtx1080/Documents/training_Data/my_data/label_img/*.jpg')):
        filename_label_list.append(label_filename) 
        
    print "The number of image need to process: ",len(filename_ori_list)
    print len(filename_ori_list)*20, "images will be generated"
    filename_ori_list.reverse()
    filename_label_list.reverse()
    
    # Augment the data and Generate the text document for loading data
    global f
    f = open('/home/iclab-gtx1080/data/train.txt','w')
    
    for i in range(1,len(filename_ori_list)+1):
        m_str = filename_ori_list.pop()
        ori_img = cv2.imread(m_str)
        m_str = filename_label_list.pop()
        label_img = cv2.imread(m_str)
        aug_data(ori_img, label_img)
        
    f.close()
    
def aug_data(ori_img, label_img):
    # Declare Global Variable
    global foto_index
    global f    
    
    # Save the original image to data
    file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
    cv2.imwrite(file_name,ori_img)
    f.write(file_name+" ")
    file_name = 'data/label_img/training_data_'+str(foto_index)+'.jpg'
    cv2.imwrite(file_name,label_img)
    f.write(file_name+"\n")
    foto_index += 1

    for i in range(2,11):
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
        tmp_ori_img = cv2.warpAffine(ori_img,M,(max_cols,max_rows))
        tmp_label_img = cv2.warpAffine(label_img,M,(max_cols,max_rows))
        
        # Save Image
        file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
        cv2.imwrite(file_name,tmp_ori_img)
        f.write(file_name+" ")
        file_name = 'data/label_img/training_data_'+str(foto_index)+'.jpg'
        cv2.imwrite(file_name,tmp_label_img)
        f.write(file_name+"\n")
        foto_index += 1


# To do grab cut algorithm, we use the labeled image to find ROI for futher work
def find_ROI(label_img):
    min_x = 999
    min_y = 999
    max_x = 0
    max_y = 0
    for y in range(0,480):
        for x in range(0,640):
            if label_img[y,x]>0:
                if x < min_x:
                    min_x = x
                if y < min_y:
                    min_y = y
                if x > max_x:
                    max_x = x
                if y > max_y:
                    max_y = y
    return [max_x,max_y,min_x,min_y]

if __name__ == "__main__":
    global foto_index
    foto_index = 1
    main()