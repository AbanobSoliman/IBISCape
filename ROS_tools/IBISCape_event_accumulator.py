#!/usr/bin/env python
# python3 data2bag_klbr_vins.py --folder /dataset-directory --output-bag ST1_klbr.bag

print ("importing libraries")

from PIL import ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv
import pandas as pd
from natsort import natsorted, ns
from progress.bar import Bar

dir_events = './events/'
events_files = os.listdir(dir_events)

height = 1024
width = 1024
sample_freq = 20

try:
    parent_dir = os.getcwd()
    path1 = os.path.join(parent_dir, "accumulated")
    os.mkdir(path1)
except OSError:
    print("Creation of the directory %s failed" % path1)
else:
    print("Successfully created the directory %s " % path1)

timestamps = open(os.path.join(parent_dir, 'accumulated.txt'), "a")

def draw_dvs_image(events_y,events_x,events_p):
    dvs_img = np.ones((height, width, 3), dtype=np.uint8) * 255
    # Blue is negative, red is positive
    dvs_img[events_y,events_x,events_p * 2 ] = 0
    dvs_img[events_y,events_x,events_p.all() or 1 ] = 0
    return dvs_img
    
events_colm = ['#timestamp_start [ns]']
events_list = pd.read_csv('./timestamps.csv', delimiter=',', usecols=events_colm).values

n_packets = int(1 / (sample_freq * (events_list[1,0]-events_list[0,0]) * 1e-9))
print("Accumulating frames with %d event packets per frame!" %(n_packets))

i = 1
events_x = []
events_y = []
events_p = []
files = natsorted(events_files, key=lambda y: y.lower())
bar = Bar('Writing_Images', max=len(files))
for file in files:
    data = np.load(os.path.join('./events/', file))
    events_x = events_x + data['x'].tolist()
    events_y = events_y + data['y'].tolist()
    events_p = events_p + data['p'].tolist()
    if i == n_packets:
        dvs_img = draw_dvs_image(np.array(events_y, dtype=int),np.array(events_x, dtype=int),np.array(events_p, dtype=bool))
        cv2.imwrite('./accumulated/%d.png' % (data['t'][-1]), dvs_img)
        timestamps.write("%d\n" % (data['t'][-1]))
        events_x = []
        events_y = []
        events_p = []
        i = 1
    i = i + 1
    bar.next()

bar.finish()    
print("Event Accumulation Done!")
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
