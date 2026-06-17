
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

            # 生成时间戳作为文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 格式: 20260417_150530_123
            
            # 创建保存目录（如果不存在）
            save_dir = "./captured_images"
            PVC_dir = os.path.join(save_dir, "PVC")
            EVA_dir = os.path.join(save_dir, "EVA")
            os.makedirs(save_dir, exist_ok = True)
            os.makedirs(PVC_dir, exist_ok = True)
            os.makedirs(EVA_dir, exist_ok = True)
            
            # 保存 RGB 图像为 PNG
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # 转换为BGR用于cv2保存
            if which_side == "PVC":
                img_path = os.path.join(PVC_dir, f"{timestamp}.png")

            elif which_side == "EVA":
                img_path = os.path.join(EVA_dir, f"{timestamp}.png")

            cv2.imwrite(img_path, img_bgr)

        return img, depth_img

