#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

# include <rclcpp/rclcpp.hpp>
# include <sensor_msgs/msg/image.hpp>
# include <sensor_msgs/msg/point_cloud2.hpp>
# include <geometry_msgs/msg/pose_stamped.hpp>

# include <image_transport/image_transport.hpp>
# include <cv_bridge/cv_bridge.h>

from threading import Lock
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
#import ros2_numpy

import cv2 as cv
from cv_bridge import CvBridge

import numpy as np

import time
import copy
import sys
import os
import math
import matplotlib
matplotlib.use('TkAgg')


###############################################################################
#  Defines
###############################################################################
global image_width
image_width = 640  # 1920
global image_height
image_height = 480  # 1080
global img_dims
img_dims = (image_width, image_height)


global img_hfov
# img_hfov = 1.3962634016 # usb webcam
img_hfov = 1.2  # runcam

DUMMY_FIELD_PREFIX = '__'

type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32,
                                                           np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}


def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list

###############################################################################
# Class
###############################################################################


class ImageDrawer(Node):
    def __init__(self):
        self.img_ = None
        self.pl_dir_ = None
        self.est_points_ = None
        self.trans_points_ = None
        self.proj_points_ = None

        self.lock_ = Lock()

        super().__init__("image_drawer")
        self.camera_sub_ = self.create_subscription(
            Image,
            "/cable_camera/image_raw",
            self.on_img_msg,
            10
        )

        self.pl_direction_sub_ = self.create_subscription(
            PoseStamped,
            "/pl_dir_computer/powerline_direction",
            self.on_pl_direction,
            10
        )

        self.points_est_sub_ = self.create_subscription(
            PointCloud2,
            "/pl_mapper/points_est",
            self.on_points_est,
            10
        )

        self.transformed_points_sub_ = self.create_subscription(
            PointCloud2,
            "/pl_mapper/transformed_points",
            self.on_transformed_points,
            10
        )

        self.projected_points_sub_ = self.create_subscription(
            PointCloud2,
            "/pl_mapper/projected_points",
            self.on_projected_points,
            10
        )

        self.drawn_img_pub_ = self.create_publisher(
            Image,
            "/drawn_image",
            10
        )

        # time.sleep(1)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.draw_image)

    def on_img_msg(self, msg):
        print("img callback")
        if self.lock_.acquire(blocking=True):
            self.img_ = msg
            self.lock_.release()

    def on_pl_direction(self, msg):
        if self.lock_.acquire(blocking=True):
            self.pl_dir_ = msg
            self.lock_.release()

    def on_points_est(self, msg):
        if self.lock_.acquire(blocking=True):
            self.est_points_ = msg
            self.lock_.release()

    def on_transformed_points(self, msg):
        if self.lock_.acquire(blocking=True):
            self.trans_points_ = msg
            self.lock_.release()

    def on_projected_points(self, msg):
        if self.lock_.acquire(blocking=True):
            self.proj_points_ = msg
            self.lock_.release()

    def draw_image(self):
        self.lock_.acquire(blocking=True)

        # if self.trans_points_ is None or self.proj_points_ is None or self.est_points_ is None or self.pl_dir_ is None or self.img_ is None:
        if self.pl_dir_ is None:
            print("PL dir is none")
            self.lock_.release()
            return
            
        if self.img_ is None:
            print("img is None")
            self.lock_.release()
            return

        print("running")

        # trans_points = self.pcl_to_numpy(self.trans_points_)
        # proj_points = self.pcl_to_numpy(self.proj_points_)
        # est_points = self.pcl_to_numpy(self.est_points_)
        pl_dir = math.atan2(
            2*(self.pl_dir_.pose.orientation.w*self.pl_dir_.pose.orientation.z +
               self.pl_dir_.pose.orientation.x*self.pl_dir_.pose.orientation.y),
            1-2*(self.pl_dir_.pose.orientation.y*self.pl_dir_.pose.orientation.y +
                 self.pl_dir_.pose.orientation.z*self.pl_dir_.pose.orientation.z)
        )
        cvb = CvBridge()
        img = cvb.imgmsg_to_cv2(self.img_, desired_encoding='passthrough')

        self.lock_.release()

        # trans_draw = self.get_draw_points(trans_points)
        # proj_draw = self.get_draw_points(proj_points)
        # est_draw = self.get_draw_points(est_points)

        p1 = [image_width/2, image_height/2]
        p2 = [0, 0]
        p2[0] = (math.sin(pl_dir) * 10) + image_width/2
        p2[1] = (math.cos(pl_dir) * 10) + image_height/2

        img = cv.cvtColor(img, cv.COLOR_BGR2RGB)

        fig, ax = plt.subplots()
        ax.imshow(img, extent=[0, image_width, 0, image_width])
        # ax.scatter(trans_draw[0], trans_draw[1], linewidth=0.000001, color='red', label='raw points')
        # ax.scatter(proj_draw[0], proj_draw[1], linewidth=0.000001, color='yellow', label='projected points')
        # ax.scatter(est_draw[0], est_draw[1], linewidth=0.000001, color='green', label='estimated points')

        xmin, xmax = ax.get_xbound()

        if(p2[0] == p1[0]):
            xmin = xmax = p1[0]
            ymin, ymax = ax.get_ybound()
        else:
            ymax = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmax-p1[0])
            ymin = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmin-p1[0])

        l = mlines.Line2D([xmin, xmax], [ymin, ymax], color='lawngreen', label='Cable yaw')
        ax.add_line(l)

        ax.legend()

        ax.imshow(img)

        fig.canvas.draw()

        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        # img is rgb, convert to opencv's default bgr
        #img = cv.cvtColor(img,cv.COLOR_RGB2BGR)

        msg = cvb.cv2_to_imgmsg(img, encoding="bgr8")

        self.drawn_img_pub_.publish(msg)

        plt.cla()
        plt.clf()
        plt.close('all')

    def get_draw_points(self, all_points):
        h_focal_length = (img_dims[0] * 0.5) / math.tan(img_hfov * 0.5)  # in pixels

        objects_dists = []
        objects_xz_angle = []
        objects_yz_angle = []

        for i in range(len(all_points)):
            current_dist = math.sqrt(
                pow(all_points[i][0], 2) + pow(all_points[i][1], 2) + pow(all_points[i][2], 2))

            objects_dists.append(current_dist)
            objects_xz_angle.append(
                math.asin(all_points[i][0] / math.sqrt(pow(all_points[i][0], 2) + pow(all_points[i][2], 2))))
            objects_yz_angle.append(
                math.asin(all_points[i][1] / math.sqrt(pow(all_points[i][1], 2) + pow(all_points[i][2], 2))))

        x_px_vec = []
        y_px_vec = []

        for i in range(len(objects_dists)):
            # horizontal pixel
            x_depth = math.sin(objects_xz_angle[i]) * objects_dists[i]
            y_depth = math.cos(objects_xz_angle[i]) * objects_dists[i]

            xy_ratio = 999999999999

            if y_depth != 0:
                xy_ratio = x_depth/y_depth

            # -1 to mirror (pinhole stuff)
            x_px_vec.append(-1 * xy_ratio * h_focal_length + image_width/2)

            # vertical pixel
            x_depth = math.sin(objects_yz_angle[i]) * objects_dists[i]
            y_depth = math.cos(objects_yz_angle[i]) * objects_dists[i]

            xy_ratio = 999999999999

            if y_depth != 0:
                xy_ratio = x_depth/y_depth

            # -1 to mirror (pinhole stuff)
            y_px_vec.append(-1 * xy_ratio * h_focal_length + image_height/2)

        x_px = image_width/2 - np.asarray(y_px_vec) + image_height/2
        y_px = image_height/2 - np.asarray(x_px_vec) + image_width/2

        return (x_px, y_px)

    def pcl_to_numpy(self, pcl_msg):
        # dtype_list = fields_to_dtype(pcl_msg.fields, pcl_msg.point_step)

        # # parse the cloud into an array
        # cloud_arr = np.fromstring(pcl_msg.data)

        # # remove the dummy fields that were added
        # cloud_arr = cloud_arr[
        #     [fname for fname, _type in dtype_list if not (fname[:len("__")] == "__")]]

        # arr = np.reshape(cloud_arr, (pcl_msg.height, pcl_msg.width))

        points = []

        n_points = int(len(pcl_msg.data)/3/4)

        for i in range(n_points):
            point = [0, 0, 0]

            for j in range(3):
                point[j] = struct.unpack("f", pcl_msg.data[i*12+j*4:i*12+(j+1)*4])[0]

            points.append(point)

        arr = np.asarray(points)

        print(arr)

        return arr


###############################################################################
# Main
###############################################################################

if __name__ == "__main__":
    rclpy.init()

    minimal_publisher = ImageDrawer()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
