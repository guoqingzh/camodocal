
import os
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
#   and vslam teams's recording from
#   \\ger\ec\proj\ha\RSG\3D_ValidationVol2\AICV_SLAM_DATA\phase4
# 
# For support on this script. please contact 
#   guoqing.zhang@intel.com
##

## data specific variable start
#input_rosbag = os.path.expanduser('~/benchmark_data/lifelong-robotics/office1-2.bag')
#multi_image_topics = ['/t265/fisheye1/image_raw','/t265/fisheye2/image_raw' ]
#odom_topics = ['/odom']
#multi_caminfo_topics = ['/t265/fisheye1/camera_info', '/t265/fisheye2/camera_info']
#suffix = "_two_fisheyes"
## data specifc variable end

## data specific variable start
input_rosbag = os.path.expanduser('~/benchmark_data/vslam-recordings/SL03')
multi_image_topics = ['/camera0/color/image_raw','/camera1/color/image_raw' ]
odom_topics = ['/odom']
multi_caminfo_topics = ['/camera0/color/camera_info', '/camera1/color/camera_info']
suffix = ""
## data specifc variable end


data_name = os.path.splitext(os.path.basename(input_rosbag))[0]+suffix
dataset_name = os.path.normpath(os.path.dirname(input_rosbag)).split(os.sep)[-1]
if dataset_name == 'lifelong-robotics':
    from rosbags.rosbag1 import Reader
else:
    from rosbags.rosbag2 import Reader

output_path = os.path.expanduser('~/converted_benchmark_data/{}/{}'.format(dataset_name,data_name))
data_path = "{}/camodocal_input_data".format(output_path)
caminfo_path = "{}/camodocal_input_caminfo".format(output_path)

odom_connections = []
multi_image_connections = []
multi_caminfo_connections = []

calib_yaml_template_pinhole = ("%YAML:1.0\n"
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

calib_yaml_template_kb = ("%YAML:1.0\n"
        "---\n"
        "model_type: KANNALA_BRANDT\n"
        "camera_name: {}\n"
        "image_width: {}\n"
        "image_height: {}\n"
        "projection_parameters:\n"
        "    k2: {}\n"
        "    k3: {}\n"
        "    k4: {}\n"
        "    k5: {}\n"
        "    mu: {}\n"
        "    mv: {}\n"
        "    u0: {}\n"
        "    v0: {}\n")



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
            if dataset_name == 'lifelong-robotics':
                msg = deserialize_ros1(rawdata, connection.msgtype)
            else:
                msg = deserialize_cdr(rawdata, connection.msgtype)
            print("Distortion model of Camera {}:".format(count)+msg.distortion_model)
            fn = '{}/camera_{}_calib.yaml'.format(caminfo_path, count)
            with open(fn, 'w') as writer:
                if msg.distortion_model in ['plumb_bob', 'brown_conrady', 'Brown Conrady', 'Plumb Bob']:
                    writer.write(calib_yaml_template_pinhole.format("'"+str(count)+"'",msg.width,msg.height, msg.d[0], msg.d[1], msg.d[2], msg.d[3], msg.k[0], msg.k[4], msg.k[2], msg.k[5] ))
                if msg.distortion_model in ['kannala_brandt4', 'Kannala Brandt4']:
                    writer.write(calib_yaml_template_kb.format("'"+str(count)+"'",msg.width,msg.height, msg.d[0], msg.d[1], msg.d[2], msg.d[3], msg.k[0], msg.k[4], msg.k[2], msg.k[5] ))
            print("Saved: {}, {}".format(fn, connection.topic))        
    print("Parsing odometry data:")
    for connection, timestamp, rawdata in reader.messages(connections=odom_connections):
        if dataset_name == 'lifelong-robotics':
            msg = deserialize_ros1(rawdata, connection.msgtype)
        else:
            msg = deserialize_cdr(rawdata, connection.msgtype)
        r = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        r_m = r.as_matrix()
        fn = '{}/pose_{}.txt'.format(data_path, timestamp)
        with open(fn, 'w') as writer:
            writer.write('{} {} {} {} {} {} {} {} {} {} {} {}'.format(r_m[0][0], r_m[0][1], r_m[0][2], r_m[1][0], r_m[1][1], r_m[1][2], r_m[2][0], r_m[2][1], r_m[2][2], msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        print("Saved: {}, {}".format(fn, connection.topic))
    print("Parsing Image data:")
    for count, image_connections in enumerate(multi_image_connections):
        for connection, timestamp, rawdata in reader.messages(connections=image_connections):
            if dataset_name == 'lifelong-robotics':
                msg = deserialize_ros1(rawdata, connection.msgtype)
            else:
                msg = deserialize_cdr(rawdata, connection.msgtype)

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
            print("Saved: {}, {}".format(fn, connection.topic))
