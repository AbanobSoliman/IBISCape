import os
import pandas as pd
import numpy as np
from natsort import natsorted, ns

path = './cam1/'
files_rename = os.listdir(path)

with open('accumulated.txt') as f:
    contents = np.array(f.readlines(), dtype=int)
    
i = 0
for file in natsorted(files_rename, key=lambda y: y.lower()):
    os.rename(os.path.join(path, file), os.path.join(path, ''.join([str(contents[i]), '.png'])))
    i = i + 1

#imu0_read = pd.read_csv('./imu0.csv')
#x = cam0_path[0,0]
#for k in range(len(imu0_read)):
#    imu0_read.loc[k, '#timestamp [ns]'] = x
#    x = x + 0.005*10**9

#imu0_read.to_csv("./imu0.csv", index=False)

