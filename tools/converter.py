
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
# Please note the followings:
# - The script is coded specifically to support benchmark data 
#   from https://lifelong-robotic-vision.github.io/dataset/scene.html
# - we only support one camera. 
# 
# For support on this script. please contact 
#   guoqing.zhang@intel.com
##


input_rosbag = os.path.expanduser('~/benchmark_data/lifelong-robotics/corridor1-1.bag')
basename = os.path.splitext(os.path.basename(input_rosbag))[0]
output_path = os.path.expanduser('~/converted_benchmark_data/{}'.format(basename))
data_path = "{}/camodocal_input_data".format(output_path)
caminfo_path = "{}/camodocal_input_caminfo".format(output_path)

multi_image_topics = ['/d400/color/image_raw','/t265/fisheye1/image_raw' ]
multi_camera_models = ['PINHOLE', 'PINHOLE']
odom_topics = ['/odom']
multi_caminfo_topics = ['/d400/color/camera_info', '/t265/fisheye1/camera_info']

odom_connections = []
multi_image_connections = []
multi_caminfo_connections = []

calib_yaml_template = ("%YAML:1.0\n"
        "---\n"
        "model_type: {}\n"
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

make_folder(data_path)
make_folder(caminfo_path)

with Reader(input_rosbag) as reader:
    for connection in reader.connections:
        if connection.topic in odom_topics:
            print("*",connection.topic, connection.msgtype)
            odom_connections.append(connection)
    
    for image_topic, caminfo_topic in zip(multi_image_topics, multi_caminfo_topics):
        for connection in reader.connections:
            if connection.topic == image_topic:    
                print("*",connection.topic, connection.msgtype)
                multi_image_connections.append([connection])    
            if connection.topic == caminfo_topic:
                print("*",connection.topic, connection.msgtype)
                multi_caminfo_connections.append([connection])

    if len(odom_connections) == 0:
        print("Error: Missing odom connections!")
        exit(0)
    if len(multi_image_connections) == 0:
        print("Error: Missing image connections!")
        exit(0)
    if len(multi_caminfo_connections) == 0:
        print("Error: missing camera info connections!")
        exit(0)

    print("Parsing camera info:")
    for count, caminfo_connections in enumerate(multi_caminfo_connections):
        for connection, timestamp, rawdata in reader.messages(connections=caminfo_connections):
            msg = deserialize_ros1(rawdata, connection.msgtype)
            fn = '{}/camera_{}_calib.yaml'.format(caminfo_path, count)
            with open(fn, 'w') as writer:
                writer.write(calib_yaml_template.format(multi_camera_models[count],"'"+str(count)+"'",msg.width,msg.height, msg.d[0], msg.d[1], msg.d[2], msg.d[3], msg.k[0], msg.k[4], msg.k[2], msg.k[5] ))
            print("Saved: {}".format(fn))        
    print("Parsing odometry data:")
    for connection, timestamp, rawdata in reader.messages(connections=odom_connections):
        msg = deserialize_ros1(rawdata, connection.msgtype)
        r = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        r_m = r.as_matrix()
        fn = '{}/pose_{}.txt'.format(data_path, timestamp)
        with open(fn, 'w') as writer:
            writer.write('{} {} {} {} {} {} {} {} {} {} {} {}'.format(r_m[0][0], r_m[0][1], r_m[0][2], r_m[1][0], r_m[1][1], r_m[1][2], r_m[2][0], r_m[2][1], r_m[2][2], msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        print("Saved: {}".format(fn))
    print("Parsing Image data:")
    for count, image_connections in enumerate(multi_image_connections):
        for connection, timestamp, rawdata in reader.messages(connections=image_connections):
            msg = deserialize_ros1(rawdata, connection.msgtype)
        
            if msg.encoding == 'rgb8':
                image_data = msg.data.reshape([msg.height, msg.step])
                image_data_3 = np.zeros([msg.height, msg.width, 3])
                image_data_3[:,:,0] = image_data[:,0::3]
                image_data_3[:,:,1] = image_data[:,1::3]
                image_data_3[:,:,2] = image_data[:,2::3]
                gray = rgb2gray(image_data_3)
            elif msg.encoding == '8UC1':
                gray = msg.data.reshape([msg.height, msg.step])/255
            else:
                print("Unsupported image format: {}".format(msg.encoding))
            fn = '{}/camera_{}_{}.png'.format(data_path, count, timestamp)
            plt.imsave( fn, gray, cmap='gray',format='png', vmin=0, vmax=1)
            print("Saved: {}".format(fn))
