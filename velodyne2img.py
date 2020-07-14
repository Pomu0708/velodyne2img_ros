import rospy
import rosparam
import numpy as np
import serial
import roslib.message

#import python_pcd
from PIL import Image
import cv2

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
import math

from numpy import *
from numpy.linalg import *
import matplotlib.pyplot as plt
from cv_bridge import CvBridge



pub = rospy.Publisher('velodyne_image/reflectance', Image, queue_size=100)

def scale_to_255(a, min, max, dtype=np.uint8):
    """ Scales an array of values from specified min, max range to 0-255
        Optionally specify the data type of the output (default is uint8)
    """
    return (((a - min) / float(max - min)) * 255).astype(dtype)

def callback(msg):

    #ExtractRingNumber=[1,32]
    header = Header()
    header.frame_id = "map" 

    data = pc2.read_points(msg)
    data = np.array(list(data))

    x = data1[:,0]
    y = data1[:,1]
    z = data1[:,2]
    #i = data1[:,3]
    d = np.sqrt(x*x+y*y)

    v_res = 1.33
    h_res = 0.25
    v_fov = [-30.67, 10.67]
    d_range = (0,100)
    y_fudge=3

    # RESOLUTION AND FIELD OF VIEW SETTINGS
    v_fov_total = -v_fov[0] + v_fov[1]

    # CONVERT TO RADIANS
    v_res_rad = v_res * (np.pi / 180)
    h_res_rad = h_res * (np.pi / 180)

    # MAPPING TO CYLINDER
    x_img = np.arctan2(y, x) / h_res_rad
    y_img = -(np.arctan2(z, d) / v_res_rad)


    # THEORETICAL MAX HEIGHT FOR IMAGE
    d_plane = (v_fov_total/v_res) / (v_fov_total* (np.pi / 180))
    h_below = d_plane * np.tan(-v_fov[0]* (np.pi / 180))
    h_above = d_plane * np.tan(v_fov[1] * (np.pi / 180))
    y_max = int(np.ceil(h_below+h_above + y_fudge))

    # SHIFT COORDINATES TO MAKE 0,0 THE MINIMUM
    x_min = -360.0 / h_res / 2
    x_img = np.trunc(-x_img - x_min).astype(np.int32)
    x_max = int(np.ceil(360.0 / h_res))

    y_min = -((v_fov[1] / v_res) + y_fudge)
    y_img = np.trunc(y_img - y_min).astype(np.int32)

    # CLIP DISTANCES
    d = np.clip(d, a_min=d_range[0], a_max=d_range[1])

    # CONVERT TO IMAGE ARRAY
    img1 = np.zeros([y_max + 1, x_max + 1], dtype=np.uint8)
    img1[y_img, x_img] = scale_to_255(d, min=d_range[0], max=d_range[1])

    #img2 = np.zeros([y_max + 1, x_max + 1], dtype=np.uint8)
    #img2[y_img, x_img] = scale_to_255(i, min=d_range[0], max=d_range[1])
    
    
    new_image1 = img1.astype(np.uint8)
    #new_image2 = img2.astype(np.uint8)

    
    ref_img = cv2.resize(new_image1, dsize=(300, 64), interpolation=cv2.INTER_LINEAR)
    #print(new_image2)
    

    

    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(ref_img, encoding="mono8")
    
    #rate = rospy.Rate(0.5) # 1hz
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

  

def outputListner():
    rospy.init_node('outputListner', anonymous=True)
    
    rospy.Subscriber("velodyne_points", PointCloud2, callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        outputListner()
    except rospy.ROSInterruptException: pass