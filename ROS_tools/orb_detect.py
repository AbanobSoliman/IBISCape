import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

image_name = './D1_e2vid.png'
image_name2 = './D1_e2vid_orb.png'
img = cv.imread(image_name,1)

# Initiate ORB detector
orb = cv.ORB_create()

# find the keypoints with ORB
kp = orb.detect(img,None)

# compute the descriptors with ORB
kp, des = orb.compute(img, kp)

# draw only keypoints location,not size and orientation
img2 = cv.drawKeypoints(img, kp, None, color=(0,255,0), flags=0)

# Plot and save
plt.imshow(img2), plt.show()
cv.imwrite(image_name2, img2)
print(len(kp))
