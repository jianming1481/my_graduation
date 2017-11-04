"""
Run DeepLab-ResNet on a given image.
"""

from __future__ import print_function

import argparse
from datetime import datetime
import os
import sys
import time

#from PIL import Image
# import PIL
from PIL import Image

import tensorflow as tf
import numpy as np

from deeplab_resnet import DeepLabResNetModel, ImageReader, decode_labels, prepare_label

# ROS
import rospy
import rospkg
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from deeplab_resnet_pkg.srv import Segmentation, SegmentationResponse
from Convert_ROS_OpenCV import ImageSubscriber


IMG_MEAN = np.array((104.00698793,116.66876762,122.67891434), dtype=np.float32)
    
NUM_CLASSES = 6
SAVE_DIR = './output/'
IMG_DIR = '/home/iclab-gtx1080/Desktop/test2.jpg'
MODEL_DIR = '/home/iclab-giga/graduate_ws/src/deeplab_resnet/snapshots/model.ckpt-200000'
show_img = 0
img = None
pred = None
sess = None
input_data = None

def get_arguments():
    """Parse all the arguments provided from the CLI.
    
    Returns:
    A list of parsed arguments.
    """
    parser = argparse.ArgumentParser(description="DeepLabLFOV Network Inference.")
    parser.add_argument("--img_path", type=str, default=IMG_DIR,
                        help="Path to the RGB image file.")
    parser.add_argument("--model_weights", type=str, default=MODEL_DIR,
                        help="Path to the file with model weights.")
    parser.add_argument("--num-classes", type=int, default=NUM_CLASSES,
                        help="Number of classes to predict (including background).")
    parser.add_argument("--save-dir", type=str, default=SAVE_DIR,
                        help="Where to save predicted mask.")
    return parser.parse_args()

def load(saver, sess, ckpt_path):
    '''Load trained weights.
    
    Args:
    saver: TensorFlow saver object.
    sess: TensorFlow session.
    ckpt_path: path to checkpoint file with parameters.
    ''' 
    saver.restore(sess, ckpt_path)
    print("Restored model parameters from {}".format(ckpt_path))

def build_resnet():
    """Create the model and start the evaluation process."""
    args = get_arguments()

    # Prepare image.
    global input_data
    input_data = tf.placeholder(tf.float32, shape=[ 480, 640, 3])
    # Convert RGB to BGR.
    img_r, img_g, img_b = tf.split(axis=2, num_or_size_splits=3, value=input_data)
    img = tf.cast(tf.concat(axis=2, values=[img_b, img_g, img_r]), dtype=tf.float32)
    # Extract mean.
    img -= IMG_MEAN 
    
    # Create network.
    net = DeepLabResNetModel({'data': tf.expand_dims(img, dim=0)}, is_training=False, num_classes=args.num_classes)

    # Which variables to load.
    restore_var = tf.global_variables()

    # Predictions.
    raw_output = net.layers['fc1_voc12']
    raw_output_up = tf.image.resize_bilinear(raw_output, tf.shape(img)[0:2,])
    raw_output_up = tf.argmax(raw_output_up, dimension=3)
    global pred
    pred = tf.expand_dims(raw_output_up, dim=3)

    
    # Set up TF session and initialize variables. 
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    global sess
    global init
    sess = tf.Session(config=config)
    init = tf.global_variables_initializer()
    
    sess.run(init)
    
    # Load weights.
    loader = tf.train.Saver(var_list=restore_var)
    load(loader, sess, args.model_weights)

# Convert the format of mask to image 
def convert_2_image(mask):
    n, h, w, c = mask.shape
    outputs = np.zeros((1, h, w, 1), dtype=np.uint8)
    for i in range(1):
      outputs[i] = np.array(mask)
    return outputs


def handle_semantic_segmentation(req):
    print('Call service!')
    global img
    global pred
    global sess
    global input_data
    img = img_scriber.get_img()
    label_image_pub = rospy.Publisher("label_img",Image,queue_size=1)

    if not img is None:
        print('Semnatic Segmentation: inside handler received img!')
        # k = cv2.waitKey(30)
        # if k == 27:   # ESC
        #     cv2.destroyAllWindows()
        # Perform inference.
        preds = sess.run(pred, feed_dict={input_data: img})
        bridge = CvBridge()
        msk = decode_labels(preds, num_classes=NUM_CLASSES)
        label_image_pub.publish(bridge.cv2_to_imgmsg(msk[0], 'rgb8'))
        # im = PIL.Image.fromarray(msk[0])
        # if not os.path.exists(SAVE_DIR):
        #     os.makedirs(SAVE_DIR)
        # im.save(SAVE_DIR + 'mask.png')
        
        # print('The output file has been saved to {}'.format(SAVE_DIR + 'mask.png'))
        #print(preds.shape)
        l_img = convert_2_image(preds)
        return SegmentationResponse(True,'Reveived image!',bridge.cv2_to_imgmsg(l_img[0]))
    else:
        print('Semnatic Segmentation: inside handler none img')
        return SegmentationResponse(False,'No image!')

    


    
if __name__ == '__main__':
    rospy.init_node('semantic_segmentation', anonymous=True)
    # Get image from realsense
    img_scriber = ImageSubscriber()
    build_resnet()

    rospy.Service('Semantic_Segmentation', Segmentation, handle_semantic_segmentation)
    
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        # img = img_scriber.get_img()
        if not img is None:
            # For debug
            cv2.imshow("resnet imshow window", img)
            k = cv2.waitKey(3)
            if k == 27:   # ESC
                cv2.destroyAllWindows()
        else:
            print('Semantic Segmentation: No received image!')
            

        # else:
        #     print('No image receive!')
        rate.sleep()
    # rospy.spin()
