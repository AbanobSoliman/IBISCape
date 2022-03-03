import os
import pandas as pd

IMG_FOLDER = 'data'  # name of your image folder
CUR_PATH = os.getcwd()  # current working directory
img_dir = os.path.join(CUR_PATH, IMG_FOLDER)  # full path to images folder
csv_data = pd.read_csv("data.csv")
images = sorted(os.listdir(img_dir))  # a list of file names in images folder
for i in range(len(csv_data)):
    # we are just checking in case file exists in folder
    os.rename(os.path.join(CUR_PATH, IMG_FOLDER, images[i]), os.path.join(CUR_PATH, IMG_FOLDER, csv_data['filename'][i]))
        
