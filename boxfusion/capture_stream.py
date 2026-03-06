"""
Dataset to stream RGB-D data from the NeRFCapture iOS App -> Cubify Transformer

Adapted from SplaTaM: https://github.com/spla-tam/SplaTAM
"""

# import numpy as np
# import time
# import torch
# import glob
# import os
# import cv2
# import torch
# import torch.nn.functional as F
# import numpy as np
# import re

# from dataclasses import dataclass

# from PIL import Image
# from scipy.spatial.transform import Rotation
# from torch.utils.data import IterableDataset

# from boxfusion.boxes import DepthInstance3DBoxes
# from boxfusion.measurement import ImageMeasurementInfo, DepthMeasurementInfo
# from boxfusion.orientation import ImageOrientation, rotate_tensor, ROT_Z
# from boxfusion.sensor import SensorArrayInfo, SensorInfo, PosedSensorInfo

# # for ros2 version
# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import Buffer, TransformListener
# from tf2_ros import TransformException
# import cv_bridge
# import numpy as np
# import queue
# import threading
# import time
# from scipy.spatial.transform import Rotation

import numpy as np
import time
import torch
import glob
import os
import cv2
import re
import threading
import queue
from dataclasses import dataclass
from PIL import Image
from scipy.spatial.transform import Rotation
from torch.utils.data import IterableDataset

# ROS1 相关导入
import rospy
import tf
import message_filters
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import CameraInfo
import cv_bridge

from boxfusion.boxes import DepthInstance3DBoxes
from boxfusion.measurement import ImageMeasurementInfo, DepthMeasurementInfo
from boxfusion.orientation import ImageOrientation, rotate_tensor, ROT_Z
from boxfusion.sensor import SensorArrayInfo, SensorInfo, PosedSensorInfo


def parse_transform_3x3_np(data):
    return torch.tensor(data.reshape(3, 3).astype(np.float32))

def parse_transform_4x4_np(data):
    return torch.tensor(data.reshape(4, 4).astype(np.float32))

def parse_size(data):
    return tuple(int(x) for x in data.decode("utf-8").strip("[]").split(", "))




T_RW_to_VW = np.array([[0, 0, -1, 0],
                       [-1,  0, 0, 0],
                       [0, 1, 0, 0],
                       [ 0, 0, 0, 1]]).reshape((4,4)).astype(np.float32)

T_RC_to_VC = np.array([[1,  0,  0, 0],
                       [0, -1,  0, 0],
                       [0,  0, -1, 0],
                       [0,  0,  0, 1]]).reshape((4,4)).astype(np.float32)

T_VC_to_RC = np.array([[1,  0,  0, 0],
                       [0, -1,  0, 0],
                       [0,  0, -1, 0],
                       [0,  0,  0, 1]]).reshape((4,4)).astype(np.float32)

def compute_VC2VW_from_RC2RW(T_RC_to_RW):
    T_vc2rw = np.matmul(T_RC_to_RW,T_VC_to_RC)
    T_vc2vw = np.matmul(T_RW_to_VW,T_vc2rw)
    return T_vc2vw

def get_camera_to_gravity_transform(pose, current, target=ImageOrientation.UPRIGHT):
    z_rot_4x4 = torch.eye(4).float()
    z_rot_4x4[:3, :3] = ROT_Z[(current, target)]
    pose = pose @ torch.linalg.inv(z_rot_4x4.to(pose))

    # This is somewhat lazy.
    fake_corners = DepthInstance3DBoxes(
        np.array([[0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0]])).corners[:, [1, 5, 4, 0, 2, 6, 7, 3]]
    fake_corners = torch.cat((fake_corners, torch.ones_like(fake_corners[..., :1])), dim=-1).to(pose)

    fake_corners = (torch.linalg.inv(pose) @ fake_corners.permute(0, 2, 1)).permute(0, 2, 1)[..., :3]
    fake_basis = torch.stack([
        (fake_corners[:, 1] - fake_corners[:, 0]) / torch.linalg.norm(fake_corners[:, 1] - fake_corners[:, 0], dim=-1)[:, None],
        (fake_corners[:, 3] - fake_corners[:, 0]) / torch.linalg.norm(fake_corners[:, 3] - fake_corners[:, 0], dim=-1)[:, None],
        (fake_corners[:, 4] - fake_corners[:, 0]) / torch.linalg.norm(fake_corners[:, 4] - fake_corners[:, 0], dim=-1)[:, None],
    ], dim=1).permute(0, 2, 1)

    # this gets applied _after_ predictions to put it in camera space.
    T = Rotation.from_euler("xz", Rotation.from_matrix(fake_basis[-1].cpu().numpy()).as_euler("yxz")[1:]).as_matrix()

    return torch.tensor(T).to(pose)

MAX_LONG_SIDE = 1024





def get_camera_to_gravity_transform(pose, current, target=ImageOrientation.UPRIGHT):
    z_rot_4x4 = torch.eye(4).float()
    z_rot_4x4[:3, :3] = ROT_Z[(current, target)]
    pose = pose @ torch.linalg.inv(z_rot_4x4.to(pose))
    fake_corners = DepthInstance3DBoxes(
        np.array([[0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0]])).corners[:, [1, 5, 4, 0, 2, 6, 7, 3]]
    fake_corners = torch.cat((fake_corners, torch.ones_like(fake_corners[..., :1])), dim=-1).to(pose)
    fake_corners = (torch.linalg.inv(pose) @ fake_corners.permute(0, 2, 1)).permute(0, 2, 1)[..., :3]
    fake_basis = torch.stack([
        (fake_corners[:, 1] - fake_corners[:, 0]) / torch.linalg.norm(fake_corners[:, 1] - fake_corners[:, 0], dim=-1)[:, None],
        (fake_corners[:, 3] - fake_corners[:, 0]) / torch.linalg.norm(fake_corners[:, 3] - fake_corners[:, 0], dim=-1)[:, None],
        (fake_corners[:, 4] - fake_corners[:, 0]) / torch.linalg.norm(fake_corners[:, 4] - fake_corners[:, 0], dim=-1)[:, None],
    ], dim=1).permute(0, 2, 1)
    T = Rotation.from_euler("xz", Rotation.from_matrix(fake_basis[-1].cpu().numpy()).as_euler("yxz")[1:]).as_matrix()
    return torch.tensor(T).to(pose)

class MultiSensorFusionROS1:
    def __init__(self, source_frame='map', target_frame='camera_link'):
        self.bridge = cv_bridge.CvBridge()
        self.result_queue = queue.Queue(maxsize=10)
        
        # TF1 监听器
        self.tf_listener = tf.TransformListener()
        self.source_frame = source_frame
        self.target_frame = target_frame

        # 使用消息过滤器进行时间同步
        self.rgb_sub = message_filters.Subscriber('/tesse/left_cam/rgb/image_raw', ROSImage)
        self.depth_sub = message_filters.Subscriber('/tesse/depth_cam/mono/image_raw', ROSImage)
        
        # 近似时间同步 (slop 为允许的时间差，单位秒)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.synced_callback)
        
        rospy.loginfo("🚀 BoxFusion ROS1 Node 启动成功")

    def synced_callback(self, rgb_msg, depth_msg):
        try:
            # 获取当前时间点的 TF 位姿
            # (trans, rot) = self.tf_listener.lookupTransform(self.source_frame, self.target_frame, rgb_msg.header.stamp)
            # 如果上面那行因为延迟报错，改用最新位姿：
            (trans, rot) = self.tf_listener.lookupTransform(self.source_frame, self.target_frame, rospy.Time(0))
            
            # 转换为 4x4 矩阵
            T_world_camera = self.tf_listener.fromTranslationRotation(trans, rot)
            
            # 转换图像
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            
            # 放入队列供 Dataset 使用
            if not self.result_queue.full():
                self.result_queue.put({
                    'rgb': rgb_image,
                    'depth': depth_image,
                    'pose': T_world_camera
                })
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF 查找失败: {e}")

    def get_synced_data(self, timeout=1.0):
        return self.result_queue.get(timeout=timeout)

class ROSDataset(IterableDataset):
    def __init__(self, cfg, has_depth=True):
        super(ROSDataset, self).__init__()
        # 初始化 ROS1 节点
        rospy.init_node('boxfusion_stream', anonymous=True)
        
        self.cfg = cfg
        self.img_height = cfg['cam']['H']
        self.img_width = cfg['cam']['W']
        self.fx = cfg['cam']['fx']
        self.fy = cfg['cam']['fy']
        self.cx = cfg['cam']['cx']
        self.cy = cfg['cam']['cy']
        self.depth_scale = cfg['cam']['png_depth_scale']
        self.has_depth = has_depth
        self.video_id = 'ros1_stream'
        self.load_arkit_depth = False

        # 启动 Fusion 处理器
        self.processor = MultiSensorFusionROS1(
            source_frame=cfg.get('ros', {}).get('map_frame', 'map'),
            target_frame=cfg.get('ros', {}).get('camera_frame', 'camera_link')
        )
        
        # 启动一个后台线程进行数据轮询 (防止阻塞主循环)
        self.spin_thread = threading.Thread(target=rospy.spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()
    
    def __len__(self):
        return 100000000
        # return 2500
        # return 1800

    def __iter__(self):
        index = 0
        while not rospy.is_shutdown():
            try:
                data = self.processor.get_synced_data(timeout=1.0)
                color_data = cv2.cvtColor(data['rgb'], cv2.COLOR_BGR2RGB)
                depth_data = data['depth'].astype(np.float32) / self.depth_scale
                pose = data['pose']

                # 缩放至模型要求的尺寸
                H, W = depth_data.shape
                color_data = cv2.resize(color_data, (W, H))

                result = dict(wide=dict())
                wide = PosedSensorInfo()
                
                # 设置相机内参
                image_info = ImageMeasurementInfo(
                    size=(self.img_width, self.img_height),
                    K=torch.tensor([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])[None]
                )
                wide.image = image_info
                result["wide"]["image"] = torch.tensor(np.moveaxis(color_data, -1, 0))[None]

                if self.has_depth:
                    depth_info = DepthMeasurementInfo(
                        size=(self.img_width, self.img_height),
                        K=torch.tensor([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])[None]
                    )
                    wide.depth = depth_info
                    depth_data = cv2.resize(depth_data, (self.img_width, self.img_height))
                    result["wide"]["depth"] = torch.tensor(depth_data)[None].float()

                # 处理重力对齐和位姿
                RT = torch.from_numpy(pose.astype(np.float32))
                wide.RT = RT[None]
                current_orientation = wide.orientation
                target_orientation = ImageOrientation.UPRIGHT

                T_gravity = get_camera_to_gravity_transform(wide.RT[-1], current_orientation, target=target_orientation)
                wide = wide.orient(current_orientation, target_orientation)

                # 重置 RT 为单位阵 (BoxFusion 在线模式通常期望以第一帧或重力对齐为参考)
                wide.RT = torch.eye(4)[None]
                wide.T_gravity = T_gravity[None]

                # 填充 GT 信息 (用于可视化)
                gt = PosedSensorInfo()
                gt.RT = RT[None]
                if self.has_depth: gt.depth = depth_info

                sensor_info = SensorArrayInfo()
                sensor_info.wide = wide
                sensor_info.gt = gt

                result["meta"] = dict(video_id=self.video_id, timestamp=index)
                result["sensor_info"] = sensor_info

                index += 1
                yield result

            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"数据处理异常: {e}")

# ... (保留原有的 ScannetDataset 和 CA1MDataset 类代码)










class ScannetDataset(IterableDataset):
    def __init__(self, cfg, has_depth=True):
        super(ScannetDataset, self).__init__()

        self.load_arkit_depth = False
        self.start = cfg['data']['start']

        self.basedir = cfg['data']['datadir']

        self.img_files = sorted(glob.glob(os.path.join(
            self.basedir, 'color', '*.jpg')), key=lambda x: int(os.path.basename(x)[:-4]))
        self.depth_paths = sorted(
            glob.glob(os.path.join(
            self.basedir, 'depth', '*.png')), key=lambda x: int(os.path.basename(x)[:-4]))
        self.load_poses(os.path.join(self.basedir, 'pose'))
        
        self.img_files=self.img_files[self.start:]
        self.depth_paths=self.depth_paths[self.start:]
        self.poses=self.poses[self.start:]

        self.frame_ids = range(0, len(self.img_files))
        self.num_frames = len(self.frame_ids)
        self.cfg = cfg
        self.img_height = cfg['cam']['H']
        self.img_width = cfg['cam']['W']
        self.K = np.array([[cfg['cam']['fx'], 0.0, cfg['cam']['cx']],
                            [0.0, cfg['cam']['fy'], cfg['cam']['cy']],
                            [0.0,0.0,1.0]])
        self.fx = cfg['cam']['fx']
        self.fy = cfg['cam']['fy']
        self.cx = cfg['cam']['cx']
        self.cy = cfg['cam']['cy']
        self.depth_scale = cfg['cam']['png_depth_scale']
        self.has_depth = has_depth
        pattern = r'scene\d{4}_\d{2}'  
        matches = re.findall(pattern, cfg['data']['datadir'])
        self.video_id = matches

    def load_poses(self, path):
        self.poses = []
        pose_paths = sorted(glob.glob(os.path.join(path, '*.txt')),
                            key=lambda x: int(os.path.basename(x)[:-4]))
        self.last_valid_pose = None
        for pose_path in pose_paths:
            with open(pose_path, "r") as f:
                lines = f.readlines()
            ls = []
            for line in lines:
                l = list(map(float, line.split(' ')))
                ls.append(l)
            c2w = np.array(ls).reshape(4, 4)

            if not np.isinf(c2w).any():
                self.last_valid_pose = c2w
            else:
                c2w = self.last_valid_pose 
            # c2w[:3, 1] *= -1
            # c2w[:3, 2] *= -1
            # c2w = torch.from_numpy(c2w).float()
            self.poses.append(c2w)

    def __len__(self):
        return self.num_frames



    def __iter__(self):
        print("Waiting for frames...")
        video_id = self.video_id
        index = 0
        while True:

            #Step1: load data
            color_path = self.img_files[index]
            depth_path = self.depth_paths[index]
            color_data = cv2.imread(color_path)

            if '.png' in depth_path:
                depth_data = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            elif '.exr' in depth_path:
                raise NotImplementedError()

            color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
            color_data = color_data 
            depth_data = depth_data.astype(np.float32) / self.depth_scale #* self.sc_factor

            H, W = depth_data.shape
            color_data = cv2.resize(color_data, (W, H))
            pose = self.poses[index]

            #Step2:try to warp the data like the original dataset    
            result = dict(wide=dict())
            wide = PosedSensorInfo()            
            
            # OK, we have a frame. Fill on the requisite data/fields.
            image_info = ImageMeasurementInfo(
                size=(self.img_width, self.img_height),
                K=torch.tensor([
                    [self.fx, 0.0, self.cx],
                    [0.0, self.fy, self.cy],
                    [0.0, 0.0, 1.0]
                ])[None])

            # print(image_info.size)

            image = np.asarray(color_data).reshape((self.img_height, self.img_width, 3))

            wide.image = image_info
            result["wide"]["image"] = torch.tensor(np.moveaxis(image, -1, 0))[None]

            if self.load_arkit_depth and not self.has_depth:
                raise ValueError("Depth was not found, you likely can only run the RGB only model with your device")

            depth_info = None            
            if self.has_depth:
                # We'll eventually ensure this is 1/4.
                depth_info = DepthMeasurementInfo(
                    size=(self.img_width, self.img_height),
                    K=torch.tensor([
                        [self.fx  , 0.0, self.cx ],
                        [0.0, self.fy , self.cy ],
                        [0.0, 0.0, 1.0]
                    ])[None])

                depth_scale = self.depth_scale
                wide.depth = depth_info

                # If I understand this correctly, it looks like this might just want the lower 16 bits?
                depth_data = cv2.resize(depth_data, (self.img_width, self.img_height))

                depth = torch.tensor(depth_data.view(dtype=np.float32).reshape((self.img_height, self.img_width)))[None].float()
                result["wide"]["depth"] = depth
                
                # desired_image_size = (4 * depth_info.size[0], 4 * depth_info.size[1])
                # wide.image = wide.image.resize(desired_image_size)

                if max(wide.image.size) > MAX_LONG_SIDE:
                    scale_factor = MAX_LONG_SIDE / max(wide.image.size)
                    # scale_factor = 1
                    new_size = (int(wide.image.size[0] * scale_factor), int(wide.image.size[1] * scale_factor))
                    wide.image = wide.image.resize(new_size)
                    result["wide"]["image"] = torch.tensor(np.moveaxis(np.array(Image.fromarray(image).resize(new_size)), -1, 0))[None]
                
            else:
                # Even for RGB-only, only support a certain long size.
                # if max(wide.image.size) > MAX_LONG_SIDE:
                # scale_factor = MAX_LONG_SIDE / max(wide.image.size)
                scale_factor = 1

                new_size = (int(wide.image.size[0] * scale_factor), int(wide.image.size[1] * scale_factor))
                wide.image = wide.image.resize(new_size)
                result["wide"]["image"] = torch.tensor(np.moveaxis(np.array(Image.fromarray(image).resize(new_size)), -1, 0))[None]


            # ARKit sends W2C?
            # While we don't necessarily care about pose, we use it to derive the orientation
            # and T_gravity.
            RT = torch.from_numpy(pose.astype(np.float32).reshape((4, 4)))
            wide.RT = RT[None]

            current_orientation = wide.orientation
            target_orientation = ImageOrientation.UPRIGHT

            T_gravity = get_camera_to_gravity_transform(wide.RT[-1], current_orientation, target=target_orientation)
            wide = wide.orient(current_orientation, target_orientation)

            '''
            Rotate IMG and Depth
            '''
            result["wide"]["image"] = rotate_tensor(result["wide"]["image"], current_orientation, target=target_orientation)
            if wide.has("depth"):
                result["wide"]["depth"] = rotate_tensor(result["wide"]["depth"], current_orientation, target=target_orientation)

            # No need for pose anymore.
            wide.RT = torch.eye(4)[None]
            wide.T_gravity = T_gravity[None]
            # print(f"T_gravity: {T_gravity}")

            gt = PosedSensorInfo()        
            gt.RT = parse_transform_4x4_np(pose)[None]
            if depth_info is not None:
                gt.depth = depth_info

            sensor_info = SensorArrayInfo()
            sensor_info.wide = wide
            sensor_info.gt = gt

            result["meta"] = dict(video_id=video_id, timestamp=index)
            result["sensor_info"] = sensor_info

            index+=1
            
            yield result



class CA1MDataset(IterableDataset):
    def __init__(self, cfg, has_depth=True):
        super(CA1MDataset, self).__init__()

        self.load_arkit_depth = False
        self.start = cfg['data']['start']

        self.basedir = cfg['data']['datadir']
        self.img_files = sorted(glob.glob(os.path.join(
            self.basedir, 'rgb', '*.png')), key=lambda x: int(os.path.basename(x)[:-4]))
        self.depth_paths = sorted(
            glob.glob(os.path.join(
            self.basedir, 'depth', '*.png')), key=lambda x: int(os.path.basename(x)[:-4]))
        self.load_poses(os.path.join(self.basedir, 'all_poses.npy'))
        
        self.img_files=self.img_files[self.start:]
        self.depth_paths=self.depth_paths[self.start:]
        self.poses=self.poses[self.start:]

        self.frame_ids = range(0, len(self.img_files))
        self.num_frames = len(self.frame_ids)
        self.cfg = cfg

        depth_intric = np.loadtxt(os.path.join(self.basedir, 'K_depth.txt')).reshape(3,3)
        self.K = np.array([[depth_intric[0,0], 0.0, depth_intric[0,2]],
                            [0.0, depth_intric[1,1], depth_intric[1,2]],
                            [0.0,0.0,1.0]])
        self.fx = self.K[0,0]
        self.fy = self.K[1,1]
        self.cx = self.K[0,2]
        self.cy = self.K[1,2]

        if self.K[0,2]< self.K[1,2]:
            self.img_height=cfg["cam"]["W"] #l
            self.img_width=cfg["cam"]["H"] #s
        else:
            self.img_height=cfg["cam"]["H"]
            self.img_width=cfg["cam"]["W"]


        self.depth_scale = cfg['cam']['png_depth_scale']
        self.has_depth = has_depth
        pattern = r'\b4\d{7}\b'  
        matches = re.findall(pattern, cfg['data']['datadir'])
        self.video_id = matches


    def load_poses(self, path):
        self.poses = np.load(path).reshape(-1,4,4)

    def __len__(self):
        return self.num_frames



    def __iter__(self):
        print("Waiting for frames...")
        video_id = self.video_id
        index = 0
        while True:

            #Step1: load data
            color_path = self.img_files[index]
            depth_path = self.depth_paths[index]
   
            color_data = cv2.imread(color_path)

            if '.png' in depth_path:
                depth_data = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            elif '.exr' in depth_path:
                raise NotImplementedError()
            
            color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
            color_data = color_data 
            depth_data = depth_data.astype(np.float32) / self.depth_scale #* self.sc_factor

            H, W = depth_data.shape
            color_data = cv2.resize(color_data, (W, H))
            pose = self.poses[index]

            #Step2:try to warp the data like the original dataset    
            result = dict(wide=dict())
            wide = PosedSensorInfo()            
            
            # OK, we have a frame. Fill on the requisite data/fields.
            image_info = ImageMeasurementInfo(
                size=(self.img_width, self.img_height),
                K=torch.tensor([
                    [self.fx, 0.0, self.cx],
                    [0.0, self.fy, self.cy],
                    [0.0, 0.0, 1.0]
                ])[None])

            image = np.asarray(color_data).reshape((self.img_height, self.img_width, 3))

            wide.image = image_info
            result["wide"]["image"] = torch.tensor(np.moveaxis(image, -1, 0))[None]

            if self.load_arkit_depth and not self.has_depth:
                raise ValueError("Depth was not found, you likely can only run the RGB only model with your device")

            depth_info = None            
            if self.has_depth:
                # We'll eventually ensure this is 1/4.
                depth_info = DepthMeasurementInfo(
                    size=(self.img_width, self.img_height),
                    K=torch.tensor([
                        [self.fx, 0.0, self.cx ],
                        [0.0, self.fy, self.cy ],
                        [0.0, 0.0, 1.0]
                    ])[None])

                depth_scale = self.depth_scale
                wide.depth = depth_info

                # If I understand this correctly, it looks like this might just want the lower 16 bits?
                depth_data = cv2.resize(depth_data, (self.img_width, self.img_height))

                depth = torch.tensor(depth_data.view(dtype=np.float32).reshape((self.img_height, self.img_width)))[None].float()
                result["wide"]["depth"] = depth

                if max(wide.image.size) > MAX_LONG_SIDE:
                    scale_factor = MAX_LONG_SIDE / max(wide.image.size)
                    # scale_factor = 1
                    new_size = (int(wide.image.size[0] * scale_factor), int(wide.image.size[1] * scale_factor))
                    wide.image = wide.image.resize(new_size)
                    result["wide"]["image"] = torch.tensor(np.moveaxis(np.array(Image.fromarray(image).resize(new_size)), -1, 0))[None]
                
            else:
                # Even for RGB-only, only support a certain long size.
                # if max(wide.image.size) > MAX_LONG_SIDE:
                # scale_factor = MAX_LONG_SIDE / max(wide.image.size)
                scale_factor = 1

                new_size = (int(wide.image.size[0] * scale_factor), int(wide.image.size[1] * scale_factor))
                wide.image = wide.image.resize(new_size)
                result["wide"]["image"] = torch.tensor(np.moveaxis(np.array(Image.fromarray(image).resize(new_size)), -1, 0))[None]


            # ARKit sends W2C?
            # While we don't necessarily care about pose, we use it to derive the orientation
            # and T_gravity.
            RT = torch.from_numpy(pose.astype(np.float32).reshape((4, 4)))
            wide.RT = RT[None]

            current_orientation = wide.orientation
            target_orientation = ImageOrientation.UPRIGHT

            T_gravity = get_camera_to_gravity_transform(wide.RT[-1], current_orientation, target=target_orientation)
            wide = wide.orient(current_orientation, target_orientation)

            '''
            Rotate IMG and Depth
            '''
            result["wide"]["image"] = rotate_tensor(result["wide"]["image"], current_orientation, target=target_orientation)
            if wide.has("depth"):
                result["wide"]["depth"] = rotate_tensor(result["wide"]["depth"], current_orientation, target=target_orientation)

            # No need for pose anymore.
            wide.RT = torch.eye(4)[None]
            wide.T_gravity = T_gravity[None]


            gt = PosedSensorInfo()        
            gt.RT = parse_transform_4x4_np(pose)[None]
            if depth_info is not None:
                gt.depth = depth_info

            sensor_info = SensorArrayInfo()
            sensor_info.wide = wide
            sensor_info.gt = gt

            result["meta"] = dict(video_id=video_id, timestamp=index)
            result["sensor_info"] = sensor_info


            index+=1
            yield result

class HM3DDataset(IterableDataset):
    def __init__(self, cfg, has_depth=True):
        super(HM3DDataset, self).__init__()

        self.load_arkit_depth = False
        self.start = cfg['data']['start']
        self.basedir = cfg['data']['datadir']

        # 辅助函数：提取文件名中的数字进行排序，避免 10.png 排在 2.png 前面
        def extract_num(path):
            basename = os.path.basename(path)
            num_str = ''.join(filter(str.isdigit, basename))
            return int(num_str) if num_str else 0

        # 获取 RGB 图片 (兼容 png 和 jpg)
        rgb_paths = glob.glob(os.path.join(self.basedir, 'rgb', '*.png')) + \
                    glob.glob(os.path.join(self.basedir, 'rgb', '*.jpg'))
        self.img_files = sorted(rgb_paths, key=extract_num)
        
        # 获取深度图 (兼容 png 和 exr)
        depth_paths = glob.glob(os.path.join(self.basedir, 'depth', '*.png')) + \
                      glob.glob(os.path.join(self.basedir, 'depth', '*.exr'))
        self.depth_paths = sorted(depth_paths, key=extract_num)

        # 加载 pose 文件
        self.load_poses(os.path.join(self.basedir, 'pose'), extract_num)
        
        self.img_files = self.img_files[self.start:]
        self.depth_paths = self.depth_paths[self.start:]
        self.poses = self.poses[self.start:]

        self.frame_ids = range(0, len(self.img_files))
        self.num_frames = len(self.frame_ids)
        self.cfg = cfg

        # 直接从 yaml 配置文件中读取内参
        self.img_height = cfg['cam']['H']
        self.img_width = cfg['cam']['W']
        self.fx = cfg['cam']['fx']
        self.fy = cfg['cam']['fy']
        self.cx = cfg['cam']['cx']
        self.cy = cfg['cam']['cy']
        self.depth_scale = cfg['cam']['png_depth_scale']
        self.has_depth = has_depth

        # 提取视频/场景 ID (例如 '00824-Dd4bFSTQ8gi')
        self.video_id = [os.path.basename(os.path.normpath(self.basedir))]

    def load_poses(self, path, key_func):
        self.poses = []
        pose_paths = sorted(glob.glob(os.path.join(path, '*.txt')), key=key_func)
        self.last_valid_pose = None
        
        # 1. 局部相机坐标系转换 (Habitat OpenGL -> OpenCV)
        # 翻转 Y 轴和 Z 轴
        C_habitat2opencv = np.eye(4)
        C_habitat2opencv[1, 1] = -1
        C_habitat2opencv[2, 2] = -1
        
        # 2. 世界坐标系转换 (Habitat Y-up -> ScanNet Z-up)
        # 绕 X 轴旋转 90 度: 将原来的 +Y 映射为 +Z，原来的 +Z 映射为 -Y
        T_Yup2Zup = np.array([
            [1.0,  0.0,  0.0, 0.0],
            [0.0,  0.0, -1.0, 0.0],
            [0.0,  1.0,  0.0, 0.0],
            [0.0,  0.0,  0.0, 1.0]
        ])

        for pose_path in pose_paths:
            with open(pose_path, "r") as f:
                lines = f.readlines()
            ls = []
            for line in lines:
                l = list(map(float, line.strip().split()))
                ls.append(l)
            c2w = np.array(ls).reshape(4, 4)

            if not np.isinf(c2w).any() and not np.isnan(c2w).any():
                # 先进行局部相机轴转换，再将整体位姿扳正到 Z-up
                c2w = T_Yup2Zup @ c2w @ C_habitat2opencv
                self.last_valid_pose = c2w
            else:
                c2w = self.last_valid_pose 
            self.poses.append(c2w)

    def __len__(self):
        return self.num_frames

    def __iter__(self):
        print(f"Waiting for frames from HM3D scene: {self.video_id}...")
        video_id = self.video_id
        index = 0
        
        # 使用 index < num_frames 防止数组越界异常
        while index < self.num_frames:
            # Step 1: load data
            color_path = self.img_files[index]
            depth_path = self.depth_paths[index]
            color_data = cv2.imread(color_path)

            if '.png' in depth_path:
                depth_data = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            elif '.exr' in depth_path:
                os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"
                depth_data = cv2.imread(depth_path, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
            else:
                raise NotImplementedError(f"Unsupported depth format: {depth_path}")

            color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
            depth_data = depth_data.astype(np.float32) / self.depth_scale 

            # 不再使用深度图原始尺寸，而是直接使用 yaml 配置中的目标尺寸缩放
            color_data = cv2.resize(color_data, (self.img_width, self.img_height))
            pose = self.poses[index]

            # Step 2: try to warp the data like the original dataset    
            result = dict(wide=dict())
            wide = PosedSensorInfo()            
            
            image_info = ImageMeasurementInfo(
                size=(self.img_width, self.img_height),
                K=torch.tensor([
                    [self.fx, 0.0, self.cx],
                    [0.0, self.fy, self.cy],
                    [0.0, 0.0, 1.0]
                ])[None])

            # 因为已经 resize 过，直接转 numpy 数组即可，去掉强制的 reshape 以防崩溃
            image = np.asarray(color_data)
            wide.image = image_info
            result["wide"]["image"] = torch.tensor(np.moveaxis(image, -1, 0))[None]

            if self.load_arkit_depth and not self.has_depth:
                raise ValueError("Depth was not found, you likely can only run the RGB only model with your device")

            depth_info = None            
            if self.has_depth:
                depth_info = DepthMeasurementInfo(
                    size=(self.img_width, self.img_height),
                    K=torch.tensor([
                        [self.fx  , 0.0, self.cx ],
                        [0.0, self.fy , self.cy ],
                        [0.0, 0.0, 1.0]
                    ])[None])

                wide.depth = depth_info
                depth_data = cv2.resize(depth_data, (self.img_width, self.img_height))
                depth = torch.tensor(depth_data.view(dtype=np.float32).reshape((self.img_height, self.img_width)))[None].float()
                result["wide"]["depth"] = depth
                
                if max(wide.image.size) > MAX_LONG_SIDE:
                    scale_factor = MAX_LONG_SIDE / max(wide.image.size)
                    new_size = (int(wide.image.size[0] * scale_factor), int(wide.image.size[1] * scale_factor))
                    wide.image = wide.image.resize(new_size)
                    result["wide"]["image"] = torch.tensor(np.moveaxis(np.array(Image.fromarray(image).resize(new_size)), -1, 0))[None]
            else:
                scale_factor = 1
                new_size = (int(wide.image.size[0] * scale_factor), int(wide.image.size[1] * scale_factor))
                wide.image = wide.image.resize(new_size)
                result["wide"]["image"] = torch.tensor(np.moveaxis(np.array(Image.fromarray(image).resize(new_size)), -1, 0))[None]

            RT = torch.from_numpy(pose.astype(np.float32).reshape((4, 4)))
            wide.RT = RT[None]

            current_orientation = wide.orientation
            target_orientation = ImageOrientation.UPRIGHT

            T_gravity = get_camera_to_gravity_transform(wide.RT[-1], current_orientation, target=target_orientation)
            wide = wide.orient(current_orientation, target_orientation)

            '''
            Rotate IMG and Depth
            '''
            result["wide"]["image"] = rotate_tensor(result["wide"]["image"], current_orientation, target=target_orientation)
            if wide.has("depth"):
                result["wide"]["depth"] = rotate_tensor(result["wide"]["depth"], current_orientation, target=target_orientation)

            wide.RT = torch.eye(4)[None]
            wide.T_gravity = T_gravity[None]

            gt = PosedSensorInfo()        
            gt.RT = parse_transform_4x4_np(pose)[None]
            if depth_info is not None:
                gt.depth = depth_info

            sensor_info = SensorArrayInfo()
            sensor_info.wide = wide
            sensor_info.gt = gt

            result["meta"] = dict(video_id=video_id, timestamp=index)
            result["sensor_info"] = sensor_info

            index += 1
            yield result
