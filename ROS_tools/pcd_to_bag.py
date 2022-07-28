#! /usr/bin/env python3

import rospy
import rosbag
import tf2_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import sys
import os
import json
import numpy as np
import tf.transformations as tf_transformations
import open3d as o3d
import json
import math
import pypcd
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_msgs.msg._TFMessage
from natsort import natsorted, ns
from tqdm import tqdm

def getTimeStampsFromDir(dir):
    '''Generates a list of files from the directory'''
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.pcd', '.ply', '.text']:
                    timestamps.append(os.path.splitext(f)[0]) 
    return timestamps

def main():

    dir_pcd = './lidar/'
    output_bag_path = "./ALOAM.bag"
    pcl_data = o3d.geometry.PointCloud()
    
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        
        pcd_files = os.listdir(dir_pcd)
        files = natsorted(pcd_files, key=lambda y: y.lower())
        timestamps = getTimeStampsFromDir(dir_pcd)
        i = 0
        for filename in tqdm(files):

            # open corresponding .pcd file
            pcl_data = o3d.io.read_point_cloud(dir_pcd+filename)
            # get data
            pcl_msg = PointCloud2()
            timestamp_nsecs = str(int(float(timestamps[i])))
            points = np.asarray(pcl_data.points, dtype="float32")
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                      # PointField('rgb', 12, PointField.UINT32, 1),
                      # PointField('rgba', 12, PointField.UINT32, 1),
                      ]
            header = Header()
            header.stamp = rospy.Time(int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )      
            header.frame_id = "/velodyne_points"
            pcl_msg = point_cloud2.create_cloud(header, fields, points)
            i += 1
            # Pusblish Pointcloud2 msg
            outbag.write("/velodyne_points", pcl_msg, rospy.Time(int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) ))

        pass



if __name__ == "__main__":

    main()
