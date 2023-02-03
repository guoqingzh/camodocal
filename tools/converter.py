
import os
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, deserialize_ros1
# create reader instance and open for reading
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

##
# modify the following variables based your case
# Please note: we only support one camera. 
# 
# For support on this script. please contact 
#   guoqing.zhang@intel.com
##

image_topics = ['/d400/color/image_raw']
input_rosbag = './cafe1-1.bag'
odom_topics = ['/odom']
output_path = "./cali_seq"
caminfo_topics = ['/d400/color/camera_info']
caminfo_path = "./caminfo"
camera_name = "id0" 

odom_connections = []
image_connections = []
caminfo_connections = []

calib_yaml_template = ("%YAML:1.0\n"
        "---\n"
        "model_type: PINHOLE\n"
        "camera_name: {}\n"
        "image_width: {}\n"
        "image_height: {}\n"
        "distortion_parameters:\n"
        "    k1: {}\n"
        "    k2: {}\n"
        "    p1: {}\n"
        "    p2: {}\n"
        "projection_parameters:\n"
        "    fx: {}\n"
        "    fy: {}\n"
        "    cx: {}\n"
        "    cy: {}\n")

def rgb2gray(rgb):
    return np.dot(rgb[...,:3]/255, [0.2989, 0.5870, 0.1140])

def make_folder(path):
    isExist = os.path.exists(path)
    if not isExist:
        os.makedirs(path)

    ls_dir = os.listdir(path)

    if len(ls_dir) != 0:
        print("Output dir {} is not empty !".format(path))
        exit(0)

make_folder(output_path)
make_folder(caminfo_path)

with Reader(input_rosbag) as reader:
    for connection in reader.connections:
        if connection.topic in odom_topics:
            print("*",connection.topic, connection.msgtype)
            odom_connections.append(connection)
        elif connection.topic in image_topics:    
            print("*",connection.topic, connection.msgtype)
            image_connections.append(connection)    
        elif connection.topic in caminfo_topics:
            print("*",connection.topic, connection.msgtype)
            caminfo_connections.append(connection)
        else:
            print(connection.topic, connection.msgtype)
    if len(odom_connections) == 0:
        print("Error: Missing odom connections!")
        exit(0)
    if len(image_connections) == 0:
        print("Error: Missing image connections!")
        exit(0)
    
    print("Parsing camera info:")
    for connection, timestamp, rawdata in reader.messages(connections=caminfo_connections):
        msg = deserialize_ros1(rawdata, connection.msgtype)
        fn = '{}/camera_{}_calib.yaml'.format(caminfo_path, camera_name)
        with open(fn, 'w') as writer:
            writer.write(calib_yaml_template.format(camera_name,msg.width,msg.height, msg.d[0], msg.d[1], msg.d[2], msg.d[3], msg.k[0], msg.k[4], msg.k[2], msg.k[5] ))
        print("Saved: {}".format(fn))        

    print("Parsing odometry data:")
    for connection, timestamp, rawdata in reader.messages(connections=odom_connections):
        msg = deserialize_ros1(rawdata, connection.msgtype)
        r = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        r_m = r.as_matrix()
        fn = '{}/pose_{}.txt'.format(output_path, timestamp)
        with open(fn, 'w') as writer:
            writer.write('{} {} {} {} {} {} {} {} {} {} {} {}'.format(r_m[0][0], r_m[0][1], r_m[0][2], r_m[1][0], r_m[1][1], r_m[1][2], r_m[2][0], r_m[2][1], r_m[2][2], msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        print("Saved: {}".format(fn))
    print("Parsing Image data:")
    for connection, timestamp, rawdata in reader.messages(connections=image_connections):
        msg = deserialize_ros1(rawdata, connection.msgtype)
        
        if msg.encoding != 'rgb8':
            print("We only support rgb8, the provided image encoding is {}".format(msg.encoding))
            exit(0)

        image_data = msg.data.reshape([msg.height, msg.step])
        image_data_3 = np.zeros([msg.height, msg.width, 3])
        image_data_3[:,:,0] = image_data[:,0::3]
        image_data_3[:,:,1] = image_data[:,1::3]
        image_data_3[:,:,2] = image_data[:,2::3]
        gray = rgb2gray(image_data_3)
        fn = '{}/camera_{}_{}.png'.format(camera_name, output_path,timestamp)
        plt.imsave( fn, gray, cmap='gray',format='png', vmin=0, vmax=1)
        print("Saved: {}".format(fn))
