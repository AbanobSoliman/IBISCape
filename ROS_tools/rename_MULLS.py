import os
from natsort import natsorted, ns

dir_pcd = './lidar/'
pcd_files = os.listdir(dir_pcd)
files = natsorted(pcd_files, key=lambda y: y.lower())
for i in range(len(files)):
    new = '%05d.pcd' %(i+1)
    os.rename(os.path.join(dir_pcd, files[i]), os.path.join(dir_pcd, new))
        
