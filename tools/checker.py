
from os import listdir
from os.path import isfile,join, exists
import numpy as np 
import xml
import xml.etree.ElementTree as ET
import csv
import sys
import decimal

num_camera=int(sys.argv[1])
mypath=sys.argv[2]


def get_timestamp(files):
    num_frames = len(files);
    timestamps = []
    for f in files:
        arr = f.split("_")
        timestamps.append(float(arr[-1][:-4])*1e-9)

    timestamps = np.array(timestamps)
    timestamps = np.sort(timestamps)
    return num_frames, timestamps

def timestamp_statistic(timestamps):
    delta = timestamps[1:] - timestamps[:-1]
    max_fps = (1/delta).max()
    min_fps = (1/delta).min()
    std_fps = (1/delta).std()
    avg_fps = (1/delta).mean()

    print("MAX FPS:", (1/delta).max())
    print("MIN FPS", (1/delta).min())
    print("STD FPS", (1/delta).std())
    print("AVG FPS", (1/delta).mean())



if num_camera == 0 or mypath=="":
    print("Please provide input paramters checker.py 2 /path/to/images")
    exit(0)

for i in range(0,num_camera):
    camfiles = [f for f in listdir(mypath) if (isfile(join(mypath,f)) and f.startswith("camera_{}".format(i)))]
    
    num_frames, timestamps = get_timestamp(camfiles)
    print("Totally frames of camera{}:{}, time:{}s, from:{}".format(i, num_frames, timestamps[-1]-timestamps[0], timestamps[0]))
    timestamp_statistic(timestamps)

posefiles = [f for f in listdir(mypath) if (isfile(join(mypath,f)) and f.startswith("pose_".format(i)))]
num_poses, timestamps = get_timestamp(posefiles)
print("Totally frames of camera{}:{}, time:{}s, from:{}".format(i, num_poses, timestamps[-1]-timestamps[0], timestamps[0]))
timestamp_statistic(timestamps)


