import os
import cv2
from datetime import datetime
from mecheye.shared import *
from mecheye.area_scan_3d_camera import *
from mecheye.area_scan_3d_camera_utils import *

class ConnectAndCaptureImages(object):                                                  # 用于 Mecheye 相机采集数据类
    def __init__(self):
        self.camera = Camera()
        self.ConnectToCamera()

    def ConnectToCamera(self):
        """
            默认连接到 0 号相机
        """
        camera_infos = Camera.discover_cameras()
        error_status = self.camera.connect(camera_infos[0])
        while not error_status.is_ok():
            show_error(error_status)
            time.sleep(5)
            error_status = self.camera.connect(camera_infos[0])
        print("3. ----------------------连接到相机成功----------------------")

    def Capture(self, which_side = None, save = False):
        """
            采集 RGB 图像、深度图和点云数据，并保存为文件
        """

        img = None

        # 采集深度图
        frame3d = Frame3D()
        show_error(self.camera.capture_3d(frame3d))
        depth_map = frame3d.get_depth_map()
        depth_img = depth_map.data()

        if save == True:

            # 采集 RGB 图像
            frame2d = Frame2D()
            show_error(self.camera.capture_2d(frame2d))
            color_map = frame2d.get_color_image()
            img = color_map.data()

        return img, depth_img

