#!/usr/bin/env python
# python3 data2bag_klbr_vins.py --folder /dataset-directory --output-bag ST1_klbr.bag

print ("importing libraries")

import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import CameraInfo
from dvs_msgs.msg import EventArray
from dvs_msgs.msg import Event
from PIL import ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv
from progress.bar import Bar
import yaml

#structure
# current_folder/davisevents0/TIMESTAMP.npz
# current_folder/davisframes0/TIMESTAMP.png
# current_folder/davisevents1/TIMESTAMP.npz
# current_folder/davisframes1/TIMESTAMP.png
# current_folder/imu0.csv
# current_folder/imu1.csv
# current_folder/gnss.csv
# current_folder/poses.csv

#setup the argument list
parser = argparse.ArgumentParser(description='Create a ROS bag using the dvs and imu data.')
parser.add_argument('--folder',  metavar='folder', nargs='?', help='Data folder')
parser.add_argument('--output-bag', metavar='output_bag',  default="output.bag", help='ROS bag file %(default)s')

#print help if no argument is specified
if len(sys.argv)<2:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

global h
h = 1024
global w
w = 1024

def getImuCsvFiles(dir):
    '''Generates a list of all csv files that start with imu'''
    imu_files = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for file in files:
                if (file[0:4] == 'left' or file[0:5] == 'right') and os.path.splitext(file)[1] == ".csv":
                    imu_files.append( os.path.join( path, file ) )
    
    return imu_files

def getCamYamlFiles(dir):
    '''Generates a list of all yaml files that start with IBISCape'''
    caminfo_files = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for file in files:
                if file[0:5] == 'calib' and os.path.splitext(file)[1] == ".yaml":
                    caminfo_files.append( os.path.join( path, file ) )
    
    return caminfo_files

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
                if folder[0:11] == "davisframes":
                    cam_folders.append(folder)
    return cam_folders
    
def getDvsFoldersFromDir(dir):
    '''Generates a list of all folders that start with cam e.g. cam0'''
    dvs_folders = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for folder in folders:                
                if folder[0:4] == "left" or folder[0:5] == "right":
                    dvs_folders.append(folder)
    return dvs_folders
    
def getDvsArrayFiles(dir):
    '''Generates a list of files from the directory'''
    dvsarrays_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.npz']:
                    dvsarrays_files.append( os.path.join( path, f ) )
                    timestamps.append(os.path.splitext(f)[0]) 
    #sort by timestamp
    sort_list = sorted(zip(timestamps, dvsarrays_files))
    dvsarrays_files = [file[1] for file in sort_list]
    return dvsarrays_files

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

def loadImageToRosMsg(filename):
    image_np0 = cv2.imread(filename)
    image_np = cv2.cvtColor(image_np0, cv2.COLOR_BGR2GRAY)
    h = image_np.shape[0]
    w = image_np.shape[1]
    
    timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
    timestamp = rospy.Time( secs=int(timestamp_nsecs[0:-9]), nsecs=int(timestamp_nsecs[-9:]) )

    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.height = h
    rosimage.width = w
    rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
    rosimage.encoding = "mono8"
    rosimage.data = image_np.tobytes()
    
    return rosimage, timestamp
    
def createDvsMessge(filename, events_data):
    rosDvsArray = EventArray()
    #events_data['t'], events_data['x'], events_data['y'], events_data['p']
    
    timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
    if timestamp_nsecs[0:-9] =='':
        seconds = 0
    else:
        seconds = timestamp_nsecs[0:-9]
    timestamp = rospy.Time( secs=int(seconds), nsecs=int(timestamp_nsecs[-9:]) )
    rosDvsArray.header.stamp = timestamp
    rosDvsArray.height = h
    rosDvsArray.width = w
    
    ttt = events_data['t']
    xxx = events_data['x']
    yyy = events_data['y']
    ppp = events_data['p'] 
    
    for i in range(len(ttt)):
        times_nsec = str(int(float(ttt[i])))
        if times_nsec[0:-9] =='':
            secondss = 0
        else:
            secondss = times_nsec[0:-9]
        times_ros = rospy.Time( int(secondss), int(times_nsec[-9:]) )  
        rosDvsArray.events.append(Event(int(xxx[i]), int(yyy[i]), times_ros, ppp[i]))
    
    return rosDvsArray, timestamp

def createCamMessage(filename):

    stream = open(filename, 'r')
    calib_data = yaml.load(stream, Loader=yaml.FullLoader)
    
    cam_info = CameraInfo()
    cam_info.width = calib_data['Camera.width']
    cam_info.height = calib_data['Camera.height']
    cam_info.header.frame_id = calib_data['Camera.type']
    cam_info.K = calib_data['LEFT.K']['data']
    cam_info.D = calib_data['LEFT.D']['data']
    cam_info.R = calib_data['LEFT.R']['data']
    cam_info.P = calib_data['LEFT.P']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    cam_info_Left = cam_info
    
    cam_info = CameraInfo()
    cam_info.width = calib_data['Camera.width']
    cam_info.height = calib_data['Camera.height']
    cam_info.header.frame_id = calib_data['Camera.type']
    cam_info.K = calib_data['RIGHT.K']['data']
    cam_info.D = calib_data['RIGHT.D']['data']
    cam_info.R = calib_data['RIGHT.R']['data']
    cam_info.P = calib_data['RIGHT.P']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    cam_info_right = cam_info
    
    return cam_info_Left, cam_info_right     
    
    
#create the bag
try:
    bag = rosbag.Bag(parsed.output_bag, 'w')
    
    #write imu data
    imufiles = getImuCsvFiles(parsed.folder)
    for imufile in imufiles:
        topic = os.path.splitext(os.path.basename(imufile))[0]
        with open(imufile, "rt") as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            for row in reader:
                imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                bag.write("/davis/{0}/imu".format(topic), imumsg, timestamp)
                
    #write caminfo data
    caminfofiles = getCamYamlFiles(parsed.folder)
    for caminfo in caminfofiles:
        dvs_left, dvs_right = createCamMessage(caminfo)
                            
    #write aps images - reconstructed (e2vid)
    #camfolders = getCamFoldersFromDir(parsed.folder)
    #for camfolder in camfolders:
    #    camdir = parsed.folder + "/{0}".format(camfolder)
    #    image_files = getImageFilesFromDir(camdir)
    #    bar = Bar('%s' %(camfolder), max=len(image_files))
    #    for image_filename in image_files:
    #        image_msg, timestamp = loadImageToRosMsg(image_filename)
    #        bag.write("/{0}/image_raw".format(camfolder), image_msg, timestamp)
    #        bar.next()
    #    bar.finish()
                
    #write dvs data
    activate = True
    eventsfolders = getDvsFoldersFromDir(parsed.folder)
    for eventsfolder in eventsfolders:
        eventsdir = parsed.folder + "/{0}".format(eventsfolder)
        event_arrays = getDvsArrayFiles(eventsdir)
        bar = Bar('%s' %(eventsfolder), max=len(event_arrays))
        for event_array in event_arrays:
            events_data = np.load(event_array)
            dvsmsg, timestamp = createDvsMessge(event_array, events_data)
            bag.write("/davis/{0}/events".format(eventsfolder), dvsmsg, timestamp)
            if activate:
                bag.write("/davis/left/camera_info", dvs_left, timestamp)
            else:
                bag.write("/davis/right/camera_info", dvs_right, timestamp)
            bar.next()
        bar.finish()
        activate = False
                
                
finally:
    bag.close()
    
print ("Conversion to ROSBAG complete!")
