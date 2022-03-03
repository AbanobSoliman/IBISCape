#!/usr/bin/env python
# python3 data2bag_klbr_vins.py --folder /dataset-directory --output-bag ST1_klbr.bag

print ("importing libraries")

import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from PIL import ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv

#structure
# dataset/cam0/TIMESTAMP.png
# dataset/camN/TIMESTAMP.png
# dataset/imu0.csv

#setup the argument list
parser = argparse.ArgumentParser(description='Create a ROS bag using the images and imu data.')
parser.add_argument('--folder',  metavar='folder', nargs='?', help='Data folder')
parser.add_argument('--output-bag', metavar='output_bag',  default="output.bag", help='ROS bag file %(default)s')

#print help if no argument is specified
if len(sys.argv)<2:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

def getImageFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    image_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    image_files.append( os.path.join( path, f ) )
                    timestamps.append(os.path.splitext(f)[0]) 
    #sort by timestamp
    sort_list = sorted(zip(timestamps, image_files))
    image_files = [file[1] for file in sort_list]
    return image_files

def getCamFoldersFromDir(dir):
    '''Generates a list of all folders that start with cam e.g. cam0'''
    cam_folders = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for folder in folders:                
                if folder[0:3] == "cam":
                    cam_folders.append(folder)
    return cam_folders

def getImuCsvFiles(dir):
    '''Generates a list of all csv files that start with imu'''
    imu_files = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for file in files:
                if file[0:3] == 'imu' and os.path.splitext(file)[1] == ".csv":
                    imu_files.append( os.path.join( path, file ) )
    
    return imu_files
    
def getGtCsvFiles(dir):
    '''Generates a list of all csv files that start with GT'''
    gt_files = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for file in files:
                if file[0:2] == 'GT' and os.path.splitext(file)[1] == ".csv":
                    gt_files.append( os.path.join( path, file ) )
    
    return gt_files

def loadImageToRosMsg(filename):
    image_np0 = cv2.imread(filename)
    image_np = cv2.cvtColor(image_np0, cv2.COLOR_BGR2GRAY)
    
    timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
    timestamp = rospy.Time( secs=int(timestamp_nsecs[0:-9]), nsecs=int(timestamp_nsecs[-9:]) )

    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.height = image_np.shape[0]
    rosimage.width = image_np.shape[1]
    rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
    rosimage.encoding = "mono8"
    rosimage.data = image_np.tobytes()
    
    return rosimage, timestamp

def createImuMessge(timestamp_int, omega, alpha):
    timestamp_nsecs = str(int(float(timestamp_int)))
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
    
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(omega[0])
    rosimu.angular_velocity.y = float(omega[1])
    rosimu.angular_velocity.z = float(omega[2])
    rosimu.linear_acceleration.x = float(alpha[0])
    rosimu.linear_acceleration.y = float(alpha[1])
    rosimu.linear_acceleration.z = float(alpha[2])
    
    return rosimu, timestamp

def createPoseMessge(timestamp_int, pos, quat):
    timestamp_nsecs = str(int(float(timestamp_int)))
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
    
    rospose = TransformStamped()
    rospose.header.stamp = timestamp
    rospose.header.frame_id = "optitrack"
    rospose.child_frame_id = "tum_camera_setup1"
    rospose.transform.translation.x = float(pos[0])
    rospose.transform.translation.y = float(pos[1])
    rospose.transform.translation.z = float(pos[2])
    rospose.transform.rotation.x = float(quat[1])
    rospose.transform.rotation.y = float(quat[2])
    rospose.transform.rotation.z = float(quat[3])
    rospose.transform.rotation.w = float(quat[0])
    
    return rospose, timestamp

#create the bag
try:
    bag = rosbag.Bag(parsed.output_bag, 'w')

    #write gt data
    posefiles = getGtCsvFiles(parsed.folder)
    for posefile in posefiles:
        with open(posefile, "rt") as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            for row in reader:
                posemsg, timestamp = createPoseMessge(row[0], row[1:4], row[10:14])
                bag.write("/vrpn_client/raw_transform", posemsg, timestamp)
    
    #write images
    camfolders = getCamFoldersFromDir(parsed.folder)
    for camfolder in camfolders:
        camdir = parsed.folder + "/{0}".format(camfolder)
        image_files = getImageFilesFromDir(camdir)
        for image_filename in image_files:
            image_msg, timestamp = loadImageToRosMsg(image_filename)
            bag.write("/{0}/image_raw".format(camfolder), image_msg, timestamp)

    #write imu data
    imufiles = getImuCsvFiles(parsed.folder)
    for imufile in imufiles:
        topic = os.path.splitext(os.path.basename(imufile))[0]
        with open(imufile, "rt") as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            for row in reader:
                imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                bag.write("/{0}".format(topic), imumsg, timestamp)
                
finally:
    bag.close()
