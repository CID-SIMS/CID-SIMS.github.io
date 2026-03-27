"""
@Author  : Yidi Zhang
@Email   : yyddzhang@gmail.com
@Date    : 2023-06-08
@Example : python data2bag.py ./CID_SIMS/office_building/floor3/floor3_1/

License:
    MIT License
"""

import cv2
import time, sys, os
from ros import rosbag
import roslib
import rospy
import tf
import numpy as np

from tf2_msgs.msg import TFMessage
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image,Imu
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
from PIL import ImageFile
from PIL import Image as ImagePIL
from cv_bridge import CvBridge
from numpy import asarray

def write_trajectory_to_rosbag(trajectory, bag):
    rospy.init_node('trajectory_writer', anonymous=True)
    for timestamp, pose in trajectory:
        tf_msg = TFMessage()
        transform_stamped = TransformStamped()
        Stamp = rospy.Time.from_sec(float(timestamp))
        transform_stamped.header.stamp = Stamp
        transform_stamped.header.frame_id = "world"
        transform_stamped.child_frame_id = "base_link_gt"
        transform_stamped.transform.translation.x = pose[0]
        transform_stamped.transform.translation.y = pose[1]
        transform_stamped.transform.translation.z = pose[2]
        transform_stamped.transform.rotation.x = pose[3]
        transform_stamped.transform.rotation.y = pose[4]
        transform_stamped.transform.rotation.z = pose[5]
        transform_stamped.transform.rotation.w = pose[6]
        tf_msg.transforms.append(transform_stamped)
        bag.write("/tf", tf_msg, transform_stamped.header.stamp)

    tf_msg_static = TFMessage()
    transform_stamped1 = TransformStamped()
    transform_stamped1.header.stamp = rospy.Time.from_sec(0.0)
    transform_stamped1.header.frame_id = "base_link_gt"
    transform_stamped1.child_frame_id = "right_cam"
    transform_stamped1.transform.translation.x = 0.0
    transform_stamped1.transform.translation.y = 0.0
    transform_stamped1.transform.translation.z = 0.0
    transform_stamped1.transform.rotation.x = 0.0
    transform_stamped1.transform.rotation.y = 0.0
    transform_stamped1.transform.rotation.z = 0.0
    transform_stamped1.transform.rotation.w = 1.0

    transform_stamped2 = TransformStamped()
    transform_stamped2.header.stamp = rospy.Time.from_sec(0.0)
    transform_stamped2.header.frame_id = "base_link_gt"
    transform_stamped2.child_frame_id = "left_cam"
    transform_stamped2.transform.translation.x = 0.0
    transform_stamped2.transform.translation.y = 0.0
    transform_stamped2.transform.translation.z = 0.0
    transform_stamped2.transform.rotation.x = 0.0
    transform_stamped2.transform.rotation.y = 0.0
    transform_stamped2.transform.rotation.z = 0.0
    transform_stamped2.transform.rotation.w = 1.0

    tf_msg_static.transforms.append(transform_stamped1)
    tf_msg_static.transforms.append(transform_stamped2)

    bag.write("/tf_static", tf_msg_static, transform_stamped.header.stamp)

def ReadIMU(filename):
    file = open(filename,'r')
    all = file.readlines()
    timestamp = []
    imu_data = []
    for f in all:
        line = f.rstrip('\n').split(' ')
        timestamp.append(line[0])
        imu_data.append(line[1:])
    return timestamp,imu_data

def ReadWheel(filename):
    file = open(filename,'r')
    all = file.readlines()
    wheel_timestamp = []
    wheel_data = []
    for f in all:
        line = f.rstrip('\n').split(' ')
        wheel_timestamp.append(line[0])
        wheel_data.append(line[1:])
    return wheel_timestamp, wheel_data


def ReadImages(data_path):
    file = open(data_path + "pose.txt", 'r')
    all = file.readlines()
    imgs_color = []
    imgs_depth = []
    for f in all:
        imgs_color.append(f.rstrip('\n').split(' ')[0])
        imgs_depth.append(f.rstrip('\n').split(' ')[0])
    file.close()
    return imgs_color, imgs_depth



def write_camera_info_to_bag(bag, Stamp):
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = Stamp
        camera_info_msg.header.frame_id = 'left_cam'
        camera_info_msg.height = 480
        camera_info_msg.width = 640
        camera_info_msg.distortion_model = 'radial-rangential'
        camera_info_msg.D = [-0.04604118637879282, 0.03505887527496214, 0.0001787943036668921, -0.00024723627967045646, 0.0]
        camera_info_msg.K = [386.52199190267083, 0.0, 326.5103569741365, 0.0, 387.32300428823663, 237.40293732598795, 0.0, 0.0, 1.0]
        camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info_msg.P = [386.52199190267083, 0.0, 326.5103569741365, 0.0, 0.0, 387.32300428823663, 237.40293732598795, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        camera_info_topic = '/camera/camera_info'
        bag.write(camera_info_topic, camera_info_msg, camera_info_msg.header.stamp)

def CreateBag(data_path):

    imutimesteps, imudata = ReadIMU(data_path + "imu.txt")
    wheeltimesteps, wheeldata = ReadWheel(data_path + "odom.txt")
    imgs_color, imgs_depth = ReadImages(data_path)

    if not os.path.exists(data_path + "data.bag"):
        os.system(r'touch %s' % (data_path + "data.bag"))
    bag = rosbag.Bag(data_path + "data.bag", 'w')

    try:
        for i in range(len(imgs_color)):
            imgpil = ImagePIL.open(data_path + "color/" + imgs_color[i] + ".png")
            Stamp = rospy.Time.from_sec(float(imgs_color[i]))
            br = CvBridge()
            data = asarray(imgpil)
            color_image_msg = br.cv2_to_imgmsg(data)
            color_image_msg.header.frame_id = "left_cam"
            color_image_msg.encoding = "rgb8"
            color_image_msg.header.stamp = Stamp
            color_image_msg.height = 480
            color_image_msg.width = 640
            bag.write('/camera/color/image_raw', color_image_msg, Stamp)

            write_camera_info_to_bag(bag, Stamp)

        for i in range(len(imgs_color)):
            imgpil = ImagePIL.open(data_path + "mask/" + imgs_color[i] + ".png")
            Stamp = rospy.rostime.Time.from_sec(float(imgs_color[i]))
            br = CvBridge()
            data = asarray(imgpil)
            mask_image_msg = br.cv2_to_imgmsg(data)
            mask_image_msg.header.frame_id = "camera"
            mask_image_msg.encoding = "rgb8"
            mask_image_msg.header.stamp = Stamp
            mask_image_msg.height = 480
            mask_image_msg.width = 640
            bag.write('/camera/aligned_mask_to_color/image', mask_image_msg, Stamp)
            
        # # for kimera (need to convert mask to 3-channel mask_viz)
        # for i in range(len(imgs_color)):
        #     imgpil = ImagePIL.open(data_path + "mask_viz/" + imgs_color[i] + ".png")
        #     Stamp = rospy.Time.from_sec(float(imgs_color[i]))
        #     br = CvBridge()
        #     data = asarray(imgpil)
        #     seg_image_msg = br.cv2_to_imgmsg(data)
        #     seg_image_msg.header.frame_id = "left_cam"
        #     seg_image_msg.encoding = "rgb8"
        #     seg_image_msg.header.stamp = Stamp
        #     seg_image_msg.height = 480
        #     seg_image_msg.width = 640
        #     bag.write('/camera/segmentation/image_raw', seg_image_msg, Stamp)
        
        for i in range(len(imgs_depth)):
            depth_image = cv2.imread(data_path + "depth/" + imgs_depth[i] + ".png", cv2.IMREAD_ANYDEPTH)
            depth_image_32f = depth_image.astype(np.float32)
            depth_image_32f /= 1000.0

            #imgpil = ImagePIL.open(data_path + "depth/" + imgs_depth[i] + ".png")
            #data = asarray(imgpil)
            #depth_array = np.array(imgpil, dtype=np.uint16)

            Stamp = rospy.Time.from_sec(float(imgs_depth[i]))
            br = CvBridge()
            depth_image_msg = br.cv2_to_imgmsg(depth_image_32f, encoding="passthrough")
            #depth_image_msg = br.cv2_to_imgmsg(data)
            depth_image_msg.header.frame_id = "left_cam"
            depth_image_msg.encoding = "32FC1"
            depth_image_msg.header.stamp = Stamp
            depth_image_msg.height = 480
            depth_image_msg.width = 640
            #depth_image_msg.data = depth_array.flatten().tolist()
            bag.write('/camera/aligned_depth_to_color/image_raw', depth_image_msg, Stamp)
            
        for i in range(len(imudata)):
            imu = Imu()
            angular_v = Vector3()
            linear_a = Vector3()
            angular_v.x = float(imudata[i][0])
            angular_v.y = float(imudata[i][1])
            angular_v.z = float(imudata[i][2])
            linear_a.x = float(imudata[i][3])
            linear_a.y = float(imudata[i][4])
            linear_a.z = float(imudata[i][5])
            imuStamp = rospy.rostime.Time.from_sec(float(imutimesteps[i]))
            imu.header.stamp = imuStamp
            imu.angular_velocity = angular_v
            imu.linear_acceleration = linear_a
            bag.write("/camera/imu",imu, imuStamp)

        for i in range(len(wheeldata)):
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.from_sec(float(wheeltimesteps[i]))
            odom_msg.pose.pose.position.x = float(wheeldata[i][0])
            odom_msg.pose.pose.position.y = float(wheeldata[i][1])
            odom_msg.pose.pose.position.z = float(wheeldata[i][2])
            odom_msg.pose.pose.orientation.x = float(wheeldata[i][3])
            odom_msg.pose.pose.orientation.y = float(wheeldata[i][4])
            odom_msg.pose.pose.orientation.z = float(wheeldata[i][5])
            odom_msg.pose.pose.orientation.w = float(wheeldata[i][6])
            odom_msg.twist.twist.linear.x = float(wheeldata[i][7])
            odom_msg.twist.twist.linear.y = float(wheeldata[i][8])
            odom_msg.twist.twist.linear.z = float(wheeldata[i][9])
            odom_msg.twist.twist.angular.x = float(wheeldata[i][10])
            odom_msg.twist.twist.angular.y = float(wheeldata[i][11])
            odom_msg.twist.twist.angular.z = float(wheeldata[i][12])
            bag.write("/odom",odom_msg, odom_msg.header.stamp)

        trajectory_file = data_path + 'pose.txt'
        trajectory = []

        with open(trajectory_file, 'r') as file:
            for line in file:
                data = line.strip().split(' ')
                timestamp = float(data[0])
                pose = [float(val) for val in data[1:]]
                trajectory.append((timestamp, pose)) 
        write_trajectory_to_rosbag(trajectory, bag)

    finally:
        bag.close()

if __name__ == "__main__":
      print(sys.argv)
      CreateBag(sys.argv[1])


