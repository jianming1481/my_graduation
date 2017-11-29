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
import time
import copy
from matplotlib import pyplot as plt


# Declare Global Variable
f = None
g = None
global foto_index
global _label

'''
    Load Image Data
'''
def load_data():
    # Declare Empty list to store the path of each class
    hand_weight_ori_list = []
    hand_weight_gt_list = []

    crayons_ori_list = []
    crayons_gt_list = []

    minions_ori_list = []
    minions_gt_list = []

    koopa_ori_list = []
    koopa_gt_list = []
    
    robots_everywhere_ori_list = []
    robots_everywhere_gt_list = []

    background_list = []

    # Search all the jpg file in directories
    ''' 1st Class: Hand Weight '''
    for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/ori_img/hand_weight/*.jpg')):
    # for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/ori_img/hand_weight/*.jpg')):
        hand_weight_ori_list.append(ori_filename)
    for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/label_img/hand_weight/*.jpg')):
    # for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/label_img/hand_weight/*.jpg')):
        hand_weight_gt_list.append(label_filename)
    hand_weight_ori_list.reverse()
    hand_weight_gt_list.reverse()

    ''' 2nd Class: Crayons '''
    for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/ori_img/crayons/*.jpg')):
    # for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/ori_img/crayons/*.jpg')):
        crayons_ori_list.append(ori_filename)
    for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/label_img/crayons/*.jpg')):
    # for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/label_img/crayons/*.jpg')):
        crayons_gt_list.append(label_filename)
    crayons_ori_list.reverse()
    crayons_gt_list.reverse()
    
    ''' 3rd Class: Minions '''
    for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/ori_img/minions/*.jpg')):
    # for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/ori_img/minions/*.jpg')):
        minions_ori_list.append(ori_filename)
    for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/label_img/minions/*.jpg')):
    # for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/label_img/minions/*.jpg')):
        minions_gt_list.append(label_filename)
    minions_ori_list.reverse()
    minions_gt_list.reverse()

    ''' 4th Class: Koopa '''
    for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/ori_img/koopa/*.jpg')):
    # for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/ori_img/koopa/*.jpg')):
        koopa_ori_list.append(ori_filename)
    for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/label_img/koopa/*.jpg')):
    # for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/label_img/koopa/*.jpg')):
        koopa_gt_list.append(label_filename)
    koopa_ori_list.reverse()
    koopa_gt_list.reverse()

    ''' 5th Class: Robots Everywhere '''
    for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/ori_img/robots_everywhere/*.jpg')):
    # for ori_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/ori_img/robots_everywhere/*.jpg')):
        robots_everywhere_ori_list.append(ori_filename)
    for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/label_img/robots_everywhere/*.jpg')):
    # for label_filename in sorted(glob.glob('/home/iclab-giga/Documents/TEST_DATA_AUG/label_img/robots_everywhere/*.jpg')):
        robots_everywhere_gt_list.append(label_filename)
    robots_everywhere_ori_list.reverse()
    robots_everywhere_gt_list.reverse()

    ''' 0 Class: background'''
    for gt_filename in sorted(glob.glob('/home/iclab-giga/Documents/training_data/my_data/ori_data/background/*')):
        background_list.append(gt_filename)
    background_list.reverse()

    total_list_num = len(hand_weight_ori_list)+len(crayons_ori_list)+len(minions_ori_list)+len(koopa_ori_list)+len(robots_everywhere_ori_list)
    # print "The number of image need to process: ", total_list_num
    # print total_list_num * 20, "images will be generated"

    return hand_weight_ori_list, crayons_ori_list, minions_ori_list,koopa_ori_list,robots_everywhere_ori_list,\
           hand_weight_gt_list, crayons_gt_list, minions_gt_list,koopa_gt_list,robots_everywhere_gt_list,\
           background_list

'''
    Select object to combine together 
'''
def object_selector(is_empty_list):
    break_condition = False
    while True:
        select_list = []
        for i in range(5):
            # If data_list already empty, assign 0 to stop pop out
            if is_empty_list[i] != 0:
                select_list.append(random.randrange(0,2))
            else:
                select_list.append(0)
        # Check if all ouput is zero
        # If all output is zero
        # re-select again
        for i in range(5):
            if select_list[i] != 0:
                break_condition = True
        if break_condition:
            break
    return select_list

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
    rect = (min_x,min_y,abs(max_x-min_x),abs(max_y-min_y))
    return rect

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
def extract_from_shadow(ori_item):
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
    ori_item = cv2.bitwise_and(ori_item,ori_item,mask = res2)
    # cv2.imshow('k-mean',ori_item)
    # while(1):
    #     k = cv2.waitKey(60)
    #     if k == 27:  # Esc key to stop
    #         break

    return ori_item

def shuffle_two_list(rgb_obj_list,label_list):
    tmp_list = zip(label_list,rgb_obj_list)
    random.shuffle(tmp_list)
    label_list, rgb_obj_list = zip(*tmp_list)
    rgb_obj_list = list(rgb_obj_list)
    label_list = list(label_list)
    return rgb_obj_list, label_list

def rotate_translate_scale(ori_item):
    max_rows, max_cols, max_channel = ori_item.shape

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
    tmp_obj_img = cv2.warpAffine(ori_item,M,(max_cols,max_rows))
    
    return tmp_obj_img

def add_obj_with_bg(obj_img, bg_img):
    # I want to put logo on top-left corner, So I create a ROI
    rows, cols, channels = obj_img.shape
    roi = bg_img[0:rows, 0:cols]

    # Now create a mask of logo and create its inverse mask also
    outputgray = cv2.cvtColor(obj_img, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(outputgray, 10, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)
    # Now black-out the area of logo in ROI
    img1_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)
    # Take only region of logo from logo image.
    img2_fg = cv2.bitwise_and(obj_img, obj_img, mask=mask)

    # Put logo in ROI and modify the main image
    dst = cv2.add(img1_bg, img2_fg)
    bg_img[0:rows, 0:cols] = dst

    return bg_img

def add_all_label_img(label,label_img,label_bg_img):
    # I want to put logo on top-left corner, So I create a ROI
    rows, cols = label_img.shape
    roi = label_bg_img[0:rows, 0:cols]

    # Now create a mask of logo and create its inverse mask also
    ret, mask = cv2.threshold(label_img, 0, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)
    # Now black-out the area of logo in ROI
    img1_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)
    # Take only region of logo from logo image.
    img2_fg = cv2.bitwise_and(label_img, label_img, mask=mask)

    # Put logo in ROI and modify the main image
    dst = cv2.add(img1_bg, img2_fg)
    label_bg_img[0:rows, 0:cols] = dst

    return label_bg_img


def multi_obj_aug(rgb_obj_list,label_list,background_img_list):
    global foto_index
    for i in range(1,41):
        # Try to change order in every loop
        random.shuffle(background_img_list)
        rgb_obj_list,label_list = shuffle_two_list(rgb_obj_list,label_list)
        # Copy list to another new list
        tmp_rgb_obj_list = rgb_obj_list[:]
        tmp_label_list = label_list[:]
        # Copy list to another new list
        # If don't use 'copy' library, the value inside old list will change when new list change
        tmp_bg_img_list = copy.deepcopy(background_img_list)
        bg_img = tmp_bg_img_list.pop()

        # Create a blank image space to storage label image
        label_bg_img = np.zeros(bg_img.shape[:2], np.uint8)
        
        while len(tmp_rgb_obj_list)!=0:
            ori_item = tmp_rgb_obj_list.pop()
            _label = tmp_label_list.pop()
            # print _label
            # _label = 127
            
                # Extract obj from shadow and make a label image
            obj_item = extract_from_shadow(ori_item)
            # Rotate, shift, scaling item
            obj_img = rotate_translate_scale(obj_item)
            label_item = cv2.cvtColor(obj_img,cv2.COLOR_BGR2GRAY)
            ret, label_img_8UC1 = cv2.threshold(label_item, 0, _label, cv2.THRESH_BINARY)
            
            
            # Add all object into background img
            bg_img = add_obj_with_bg(obj_img,bg_img)
            label_bg_img = add_all_label_img(_label,label_img_8UC1,label_bg_img)
        # Save list for training
        f = open('/home/iclab-giga/graduate_ws/src/data_augmentation/data/train.txt','a')
        # f = open('/home/iclab-giga/Documents/TEST_DATA_AUG/train.txt','a')
        g = open('/home/iclab-giga/graduate_ws/src/data_augmentation/data/val.txt','a')
        # g = open('/home/iclab-giga/Documents/TEST_DATA_AUG/val.txt','a')
        # Save Image
        if i>30:
            file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
            # file_name = '/home/iclab-giga/Documents/TEST_DATA_AUG/gen/ori_img/training_data_'+str(foto_index)+'.jpg'
            cv2.imwrite(file_name,bg_img)
            g.write(file_name+" ")
            file_name = 'data/label_img/training_data_'+str(foto_index)+'.png'
            # file_name = '/home/iclab-giga/Documents/TEST_DATA_AUG/gen/label_img/training_data_'+str(foto_index)+'.png'
            cv2.imwrite(file_name,label_bg_img)
            g.write(file_name+"\n")
        else:
            file_name = 'data/ori_img/training_data_'+str(foto_index)+'.jpg'
            # file_name = '/home/iclab-giga/Documents/TEST_DATA_AUG/gen/ori_img/training_data_'+str(foto_index)+'.jpg'
            cv2.imwrite(file_name,bg_img)
            f.write(file_name+" ")
            file_name = 'data/label_img/training_data_'+str(foto_index)+'.png'
            # file_name = '/home/iclab-giga/Documents/TEST_DATA_AUG/gen/label_img/training_data_'+str(foto_index)+'.png'
            cv2.imwrite(file_name,label_bg_img)
            f.write(file_name+"\n")

        # cv2.imshow('res',bg_img)
        # while(1):
        #     k = cv2.waitKey(60)
        #     if k == 27:  # Esc key to stop
        #         break
        foto_index+=1
        f.close()
        g.close()

def main():
    # Load the Path of data into a list
    hand_weight_ori_list, crayons_ori_list, minions_ori_list,koopa_ori_list,robots_everywhere_ori_list,\
    hand_weight_gt_list, crayons_gt_list, minions_gt_list,koopa_gt_list,robots_everywhere_gt_list,\
    background_list = load_data()
    total_list_num = len(hand_weight_ori_list)+len(crayons_ori_list)+len(minions_ori_list)+len(koopa_ori_list)+len(robots_everywhere_ori_list)

    # Generate the List for Neural Netwrok to Load Data 
    global _label
    _label = 255

    # how many numbers of images do you have in one class
    img_numbers = 1
    index = 1

    # The list to restorage the path of image
    rgb_image_list = []
    label_image_list = []
    background_img_list = []

    # Read Image from List to Augment Data
    while total_list_num!=0:
        select_list = object_selector([len(hand_weight_ori_list), len(crayons_ori_list), len(minions_ori_list), len(koopa_ori_list), len(robots_everywhere_ori_list)])
        label_list = []

        if select_list[0]:
            rgb_image_list.append(hand_weight_ori_list.pop())
            label_image_list.append(hand_weight_gt_list.pop())
            label_list.append(1)
        if select_list[1]:
            rgb_image_list.append(crayons_ori_list.pop())
            label_image_list.append(crayons_gt_list.pop())
            label_list.append(2)
        if select_list[2]:
            rgb_image_list.append(minions_ori_list.pop())
            label_image_list.append(minions_gt_list.pop())
            label_list.append(3)
        if select_list[3]:
            rgb_image_list.append(koopa_ori_list.pop())
            label_image_list.append(koopa_gt_list.pop())
            label_list.append(4)
        if select_list[4]:
            rgb_image_list.append(robots_everywhere_ori_list.pop())
            label_image_list.append(robots_everywhere_gt_list.pop())
            label_list.append(5)
        print select_list
        print label_list
        index = 0
        
        # The rgb object cutted from original image
        rgb_obj_list = []
        
        while len(rgb_image_list)!=0:
            m_str = rgb_image_list.pop()
            ori_img = cv2.imread(m_str)
            m_str = label_image_list.pop()
            label_img = cv2.imread(m_str)

            """
                Use the Original Data to Augment Data
                Using IR_Img(Depth) to Find the ROI for Grab Cut Algorithm
            """
            rect = find_ROI(label_img)
            rgb_obj_list.append(my_grab_cut(ori_img,rect))

        while len(background_list)!=0:
            m_str = background_list.pop()
            background_img = cv2.imread(m_str)
            background_img_list.append(background_img)

        multi_obj_aug(rgb_obj_list,label_list,background_img_list)
        total_list_num = len(hand_weight_ori_list)+len(crayons_ori_list)+len(minions_ori_list)+len(koopa_ori_list)+len(robots_everywhere_ori_list)

    print("\n==================Finish data augmentation==================\n")

if __name__ == "__main__":
    global foto_index
    foto_index = 4001
    main()