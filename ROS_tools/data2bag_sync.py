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
import cv2 as cv
from cv_bridge import CvBridge
bridge = CvBridge()

# Reading Sequence
parent_dir = os.getcwd()
path1 = parent_dir
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

imu = pd.read_csv('other_sensors/imu_sync.csv', delimiter=';')
rgb = pd.read_csv('rgb/timestamps.csv', delimiter=';')
seg = pd.read_csv('semantic/timestamps.csv', delimiter=';')
dpt = pd.read_csv('depth/timestamps.csv', delimiter=';')
dvs = pd.read_csv('davis/timestamps.csv', delimiter=';')

# Writing Bag

def main():
  if len(sys.argv) < 2:
    print("Usage: {} dataset_name".format(sys.argv[0]))
    exit(1)

  file_name = sys.argv[1]

  with rosbag.Bag('{}.bag'.format(file_name), 'w') as bag:
    bar = Bar('RGB', max=len(rgb_images))
    for i in range(len(rgb_images)):
      m_img = Image()
      m_img.header.stamp = rospy.Time(float(rgb['#timestamp [ns]'][i]*10**-9)) 
      cv_image = cv.cvtColor(rgb_images[i], cv.COLOR_BGR2GRAY)
      encoding = "mono8"
      image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        
      bag.write('/cam0/image_raw', image_message, m_img.header.stamp)
      bar.next()
      
    bar.finish()
    
    bar = Bar('Segmentation', max=len(sseg_images))
    for i in range(len(sseg_images)):
      m_img = Image()
      m_img.header.stamp = rospy.Time(float(seg['#timestamp [ns]'][i]*10**-9)) 
      cv_image = sseg_images[i]
      encoding = "bgr8"
      image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        
      bag.write('/cam1/image_raw', image_message, m_img.header.stamp)
      bar.next()
      
    bar.finish()
    
    bar = Bar('Depth', max=len(depth_images))
    for i in range(len(depth_images)):
      m_img = Image()
      m_img.header.stamp = rospy.Time(float(dpt['#timestamp [ns]'][i]*10**-9)) 
      cv_image = cv.cvtColor(depth_images[i], cv.COLOR_BGR2GRAY)
      encoding = "mono8"
      image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        
      bag.write('/cam2/image_raw', image_message, m_img.header.stamp)
      bar.next()
      
    bar.finish()
    
    bar = Bar('DAVIS', max=len(dvs_images))
    for i in range(len(dvs_images)):
      m_img = Image()
      m_img.header.stamp = rospy.Time(float(dvs['#timestamp [ns]'][i]*10**-9)) 
      cv_image = dvs_images[i]
      encoding = "bgr8"
      image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        
      bag.write('/cam3/image_raw', image_message, m_img.header.stamp)
      bar.next()
      
    bar.finish()

    bar = Bar('IMU', max=len(imu['#timestamp [ns]']))
    for row in range(imu.shape[0]):
      timestamp = rospy.Time(float(imu['#timestamp [ns]'][row]*10**-9))
      imu_msg = Imu()
      imu_msg.header.seq = row
      imu_msg.header.frame_id = "imu0"
      imu_msg.header.stamp = timestamp
      imu_msg.angular_velocity.x = float(imu['w_RS_S_x [rad s^-1]'][row])
      imu_msg.angular_velocity.y = float(imu['w_RS_S_y [rad s^-1]'][row])
      imu_msg.angular_velocity.z = float(imu['w_RS_S_z [rad s^-1]'][row])
      imu_msg.linear_acceleration.x = float(imu['a_RS_S_x [m s^-2]'][row])
      imu_msg.linear_acceleration.y = float(imu['a_RS_S_y [m s^-2]'][row])
      imu_msg.linear_acceleration.z = float(imu['a_RS_S_z [m s^-2]'][row])
      
      bag.write("/imu0", imu_msg, timestamp)
      bar.next()

    bar.finish()
    
    bag.close()  

if __name__ == "__main__":
  main()
