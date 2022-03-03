# coding:utf-8
#!/usr/bin/env python

# Copyright (c) 2021 IBISC Laborartory, Pelvoux, France

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import matplotlib.pyplot as mpplot
import matplotlib.image as mpimg
import cv2 as cv
import glob
import os

# Don't forget to change the sequence name
parent_dir = os.getcwd()
path1 = os.path.join(parent_dir, "Async_Calib_Sequence_i_Town03") # Change sequence name here!
path2 = os.path.join(path1, "davis/frames")
path3 = os.path.join(path1, "depth/frames")
path4 = os.path.join(path1, "rgb/frames")
path5 = os.path.join(path1, "semantic/frames/")

SSeg_path = glob.glob(os.path.join(path5,'*.png'))
DVS_path = glob.glob(os.path.join(path2,'*.png'))
rgb_path = glob.glob(os.path.join(path4,'*.png'))
depth_path = glob.glob(os.path.join(path3,'*.png'))

def load_images_from_folder(folder):
    images = []
    for filename in sorted(folder):
        img = cv.imread(filename)
        images.append(img)
    return images

dvs_images = load_images_from_folder(DVS_path)
sseg_images = load_images_from_folder(SSeg_path)
rgb_images = load_images_from_folder(rgb_path)
depth_images = load_images_from_folder(depth_path)

print ('RGB Total frames:', len(rgb_images))
print ('Depth Total frames:', len(depth_images))
print ('Semantic Segmentation Total frames:', len(sseg_images))
print ('DVS Total frames:', len(dvs_images))

mpplot.ion()
for x in range(len(rgb_images)):
    mpplot.subplot(2, 2, 1)
    mpplot.imshow(rgb_images[x])
    mpplot.subplot(2, 2, 2)
    mpplot.imshow(depth_images[x])
    mpplot.subplot(2, 2, 3)
    mpplot.imshow(sseg_images[x])
    mpplot.subplot(2, 2, 4)
    mpplot.imshow(dvs_images[x])
    mpplot.show()
    mpplot.pause(0.000001)
    mpplot.clf()
mpplot.close()
