#!/usr/bin/env python
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from dvs_msgs.msg import EventArray
from dvs_msgs.msg import Event
from PIL import ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv
from progress.bar import Bar



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

eventsdir = './events'
event_arrays = getDvsArrayFiles(eventsdir)
for event_array in event_arrays:
    events_data=np.load(event_array)
    t = events_data['t']
    print(t, len(t))
    for it in range(1, len(t)-1):
        if t[it] < t[it-1]:
            print('now %d latest %d' %(t[it], t[it-1]))
            break
