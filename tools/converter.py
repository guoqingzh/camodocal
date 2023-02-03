
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
##
image_topics = ['/d400/color/image_raw']
input_rosbag = './cafe1-1.bag'
odom_topics = ['/odom']
output_path = "./data"


odom_connections = []
image_connections = []

def rgb2gray(rgb):
    return np.dot(rgb[...,:3]/255, [0.2989, 0.5870, 0.1140])

isExist = os.path.exists(output_path)
if not isExist:
    os.makedirs(output_path)

ls_dir = os.listdir(output_path)

if len(ls_dir) != 0:
    print("Output dir {} is not empty !".format(output_path))
    exit(0)

with Reader(input_rosbag) as reader:
    for connection in reader.connections:
        if connection.topic in odom_topics:
            print("*",connection.topic, connection.msgtype)
            odom_connections.append(connection)
        elif connection.topic in image_topics:    
            print("*",connection.topic, connection.msgtype)
            image_connections.append(connection)    
        else:
            print(connection.topic, connection.msgtype)
        
    if len(odom_connections) == 0:
        print("Error: Missing odom connections!")
        exit(0)
    if len(image_connections) == 0:
        print("Error: Missing image connections!")
        exit(0)
    
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
        fn = '{}/camera_0_{}.png'.format(output_path,timestamp)
        plt.imsave( fn, gray, cmap='gray',format='png', vmin=0, vmax=1)
        print("Saved: {}".format(fn))
