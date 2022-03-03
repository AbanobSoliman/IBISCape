#!/usr/bin/env python

import numpy as np
from progress.bar import Bar
import time, sys, os, glob
from ros import rosbag
import rospy
import roslib
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Imu, Image, NavSatFix
import pandas as pd
from PIL import ImageFile
import cv2

# Reading Sequence
parent_dir = os.getcwd()
path4 = os.path.join(parent_dir, "rgb/frames")
path5 = os.path.join(parent_dir, "gray")
os.mkdir(path5)
path6 = os.path.join(path5, "frames")
os.mkdir(path6)

    
rgb_path = glob.glob(os.path.join(path4,'*.png'))


def load_images_from_folder(folder):
    images = []
    for filename in sorted(folder):
        img = cv2.imread(filename, cv2.IMREAD_COLOR)
        images.append(img)
    return images

rgb_images = load_images_from_folder(rgb_path)

print ('RGB Total frames:', len(rgb_images))

for rgb_image in rgb_images:
    img_gray = cv2.cvtColor(rgb_images, cv2.COLOR_RGB2GRAY)
    #gray = cv2.bilateralFilter(img_gray,9,75,75)
    #thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 11)
    outputfile = 'gray/frames/%d.jpg' % (rgb_images.name)
    cv2.imwrite(outputfile, img_gray)
