import os
import pandas as pd
import numpy as np
from natsort import natsorted, ns

path = './cam1/'
files_rename = os.listdir(path)

fp3d_list = ['#timestamp [ns]']
cam0_path = pd.read_csv('./cam0.csv', delimiter=',', usecols=fp3d_list).values

i = 0
for file in natsorted(files_rename, key=lambda y: y.lower()):
    os.rename(os.path.join(path, file), os.path.join(path, ''.join([str(cam0_path[i,0]), '.png'])))
    i = i + 1

#imu0_read = pd.read_csv('./imu0.csv')
#x = cam0_path[0,0]
#for k in range(len(imu0_read)):
#    imu0_read.loc[k, '#timestamp [ns]'] = x
#    x = x + 0.005*10**9

#imu0_read.to_csv("./imu0.csv", index=False)

