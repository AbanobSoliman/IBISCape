#!/usr/bin/python
# -*- coding: utf-8 -*-

from PIL import ImageFile, Image, ImageEnhance
import time
import sys
import os
import glob
import argparse
import cv2
import numpy as np
import csv
import pandas as pd
from natsort import natsorted, ns
from progress.bar import Bar

global drop
drop = 0


def mask_create(events_number):
    mask = [True]
    count = 1
    for i in range(1, events_number):
        if count == drop:
            mask.append(True)
            count = 1
        else:
            mask.append(False)
        count += 1
    return mask


def write_accum_events(
    events_t,
    events_x,
    events_y,
    events_p,
    hot_pixels):
    ev_list = np.transpose(np.array([events_t,events_x,events_y,events_p])).tolist()
    ev_df = pd.DataFrame(ev_list, columns =['t','x','y','p'])
    ev_df.loc[~((ev_df['x'].isin(hot_pixels[:,0])) & (ev_df['y'].isin(hot_pixels[:,1])))]
    np.savetxt(filt_events, corresponding.values, fmt='%.12f %d %d %d', delimiter=' ')

parent_dir = os.getcwd()
filt_events = open(os.path.join(parent_dir, 'filt_events.txt'), 'a')
hot_pixels = np.loadtxt(parent_dir+"/hot_pixels.txt", comments="#", delimiter=", ", unpack=False)

dir_events = './events/'
events_files = natsorted(os.listdir(dir_events), key=lambda y: \
                         y.lower())

bar = Bar('Writing_Events', max=len(events_files))
for file in events_files:
    data = np.load(os.path.join('./events/', file))
    events_t = data['t'] * 1e-9
    events_x = data['x']
    events_y = data['y']
    events_p = data['p']
    mask = mask_create(len(events_t))
    if drop == 0:
        write_accum_events(events_t, events_x, events_y,
                       events_p,hot_pixels)
    else:
        write_accum_events(events_t[mask], events_x[mask], events_y[mask],
                       events_p[mask],hot_pixels)
    bar.next()

bar.finish()
print ('Event dropping Done!')
