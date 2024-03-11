#!/usr/bin/env python3
import rclpy
import ros2_numpy
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2, CameraInfo, Image
from laser_geometry import LaserProjection
from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge
import cv2


class Laser2Point(Node):
    def __init__(self):
        super().__init__('laser2point')
        self.get_logger().info('Init laser projection to camera image')

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info = None
        self.camera_image = None

        self.laserProj = LaserProjection()

        self.laserSub = self.create_subscription(
            LaserScan, '/scan', self.point_cloud_callback, 10)

        self.pcPub = self.create_publisher(
            PointCloud2, '/my_points', 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10)

        self.camera_img_sub = self.create_subscription(
            Image, '/image_raw', self.camera_image_callback, 10)

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_model.fromCameraInfo(msg)
        self.camera_info = msg

    def camera_image_callback(self, msg: Image):
        self.camera_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")

    def point_cloud_callback(self, msg: PointCloud2):
        if self.camera_image is None or self.camera_info is None:
            self.get_logger().info('Waiting for camera info or image....')
            return

        pc = self.laserProj.projectLaser(msg)
        data_dict = ros2_numpy.numpify(pc)
        points = data_dict['xyz']
        self.get_logger().info(
            '\n3Dcoordinate:{} number of points={}'. format(np.around(points, decimals=2), points.shape[0]))
        self.pcPub.publish(pc)

        image_with_points = self.camera_image.copy()

        for point in points:
            u, v = self.camera_model.project3dToPixel(
                (point[0], point[1], 1.0))

            if np.isnan(u) or np.isnan(v) or u >= image_with_points.shape[0] or u < 0 or v >= image_with_points.shape[1] or v < 0:
                # self.get_logger().info('{}\t\t{}\t\t{}\t\t u={}, v={}'.format(
                #     point[0], point[1], point[2], u, v))
                continue

            cv2.circle(image_with_points, (int(u), int(v)), 1, (0, 255, 0), 1)

        self.get_logger().info('u = {}, v = {}'.format(u, v))

        cv2.imshow('Projected Points', image_with_points)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    laser2point = Laser2Point()
    rclpy.spin(laser2point)
    laser2point.destroy_node()
    rclpy.shutdown()


main()
