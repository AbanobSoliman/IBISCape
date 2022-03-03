import os
import pandas as pd
import numpy as np

path0 = './cam0/'
files_rename0 = os.listdir(path0)

path1 = './cam1/'
files_rename1 = os.listdir(path1)

stamps = np.loadtxt('cam0.txt')
if len(stamps.shape) == 2:
    stamps = stamps[:, 1]

i = 0
for file in sorted(files_rename0):
    os.rename(os.path.join(path0, file), os.path.join(path0, ''.join([str(round(stamps[i]*1e9)), '.png'])))
    i = i + 1

i = 0
for file in sorted(files_rename1):
    os.rename(os.path.join(path1, file), os.path.join(path1, ''.join([str(round(stamps[i]*1e9)), '.png'])))
    i = i + 1

A = (stamps*1e9).astype(int)
np.savetxt('IBISCape.txt', A, fmt='%d')

#imu0_read = pd.read_csv('./imu0.csv')
#x = cam0_path[0,0]
#for k in range(len(imu0_read)):
#    imu0_read.loc[k, '#timestamp [ns]'] = x
#    x = x + 0.005*10**9

#imu0_read.to_csv("./imu0.csv", index=False)

