import os
import pandas as pd
import numpy as np

path0 = './cam0/'
files_rename0 = os.listdir(path0)

path1 = './cam1/'
files_rename1 = os.listdir(path1)

t0 = 298039658778
dt = 50
A = []

i = 0
for file in sorted(files_rename0):
    A.append(t0 + (i * dt))
    os.rename(os.path.join(path0, file), os.path.join(path0, ''.join([str(A[-1]), '.png'])))
    i = i + 1

i = 0
for file in sorted(files_rename1):
    os.rename(os.path.join(path1, file), os.path.join(path1, ''.join([str(A[i]), '.png'])))
    i = i + 1

a = np.array(A)
np.savetxt('IBISCape.txt', a, fmt='%d')

#imu0_read = pd.read_csv('./imu0.csv')
#x = cam0_path[0,0]
#for k in range(len(imu0_read)):
#    imu0_read.loc[k, '#timestamp [ns]'] = x
#    x = x + 0.005*10**9

#imu0_read.to_csv("./imu0.csv", index=False)

