#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pika Sense 设备类，提供对Pika Sense设备的访问接口
"""

import time
import logging
import threading
from .serial_comm import SerialComm

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('pika.sense')

class Sense:
    """
    Pika Sense设备类，提供对Pika Sense设备的访问接口
    
    参数:
        port (str): 串口设备路径，默认为'/dev/ttyUSB0'
    """
    
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        json_pattern = r'\{\r\n"Command":[0-1],\r\n"AS5047":\{\r\n"angle":[^\r\n]*,\r\n"rad":[^\r\n]*\r\n\},\r\n\r\n"IMU":\{\r\n"acc":\[[^\r\n]*\],\r\n"gyr":\[[^\r\n]*\],\r\n"pitch":[^\r\n]*,\r\n"roll":[^\r\n]*,\r\n"yaw":[^\r\n]*\r\n\}\r\n\r\n\}'
        self.serial_comm = SerialComm(port=port, json_pattern=json_pattern)
        self.is_connected = False
        self.data_lock = threading.Lock()
        
        # 编码器数据
        self.encoder_data = {
            'angle': 0.0,
            'rad': 0.0
        }
        
        # 命令状态
        self.command_state = 0
        
        # 鱼眼相机索引
        self.fisheye_camera_index = 0
        
        # realsense相机序列号
        self.realsense_serial_number = None
        
        # 相机分辨率和帧率
        self.camera_width=1280
        self.camera_height=720
        self.camera_fps=30
        
        # 相机对象，延迟初始化
        self._fisheye_camera = None
        self._realsense_camera = None
        
        # Vive Tracker对象，延迟初始化
        self._vive_tracker = None
        self._vive_tracker_config = None
        self._vive_tracker_lh = None
        self._vive_tracker_args = None
    
    def connect(self):
        """
        连接Pika Sense设备
        
        返回:
            bool: 连接是否成功
        """
        if self.is_connected:
            logger.warning("设备已经连接")
            return True
        
        # 连接串口
        if not self.serial_comm.connect():
            logger.error("连接设备失败")
            return False
        
        # 启动数据读取线程
        self.serial_comm.start_reading_thread(callback=self._data_callback)
        self.is_connected = True
        logger.info(f"成功连接到Pika Sense设备: {self.port}")
        
        # 等待初始数据
        time.sleep(0.5)
        return True
    
    def disconnect(self):
        """
        断开Pika Sense设备连接
        """
        if not self.is_connected:
            return
        
        # 断开串口连接
        self.serial_comm.disconnect()
        self.is_connected = False
        logger.info(f"已断开Pika Sense设备连接: {self.port}")
        
        # 断开相机连接
        if self._fisheye_camera:
            try:
                self._fisheye_camera.disconnect()
            except:
                pass
          
        if self._realsense_camera:
            try:
                self._realsense_camera.disconnect()
            except:
                pass
                
        # 断开Vive Tracker连接
        if self._vive_tracker:
            try:
                self._vive_tracker.disconnect()
            except:
                pass
    
    def _data_callback(self, data):
        """
        数据回调函数，处理接收到的JSON数据
        
        参数:
            data (dict): 接收到的JSON数据
        """
        try:
            with self.data_lock:

                # 处理编码器数据
                if 'AS5047' in data:
                    encoder = data['AS5047']
                    self.encoder_data = {
                        'angle': encoder.get('angle', 0.0),
                        'rad': encoder.get('rad', 0.0)
                    }
                
                # 处理命令状态
                if 'Command' in data:
                    self.command_state = data['Command']
        except Exception as e:
            logger.error(f"处理数据回调异常: {e}")
    
    def get_encoder_data(self):
        """
        获取编码器数据
        
        返回:
            dict: 编码器数据，包含angle、rad字段
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认编码器数据")
        
        with self.data_lock:
            return self.encoder_data.copy()
    
    def get_command_state(self):
        """
        获取命令状态
        
        返回:
            int: 命令状态，0或1
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认命令状态")
        
        with self.data_lock:
            return self.command_state
    
    def set_camera_param(self,camera_width,camera_height,camera_fps):
        '''
        设置相机分辨率和帧率
        
        参数:
            camera_width (int): 相机宽度
            camera_height (int): 相机高度
            camera_fps (int): 相机帧率
        '''
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fps = camera_fps
    
    def set_fisheye_camera_index(self,index):
        '''
        设置鱼眼相机的索引
        
        参数:
            index (int): 鱼眼相机索引
        '''
        self.fisheye_camera_index = index
        
    def set_realsense_serial_number(self,serial_number):
        '''
        设置realsense相机序列号
        
        参数:
            serial_number (str): realsense相机序列号
        '''
        self.realsense_serial_number = serial_number
    
    def set_vive_tracker_config(self, config_path=None, lh_config=None, args=None):
        '''
        设置Vive Tracker配置
        
        参数:
            config_path (str, optional): 配置文件路径
            lh_config (str, optional): 灯塔配置
            args (list, optional): 其他pysurvive参数
        '''
        self._vive_tracker_config = config_path
        self._vive_tracker_lh = lh_config
        self._vive_tracker_args = args
        
    def get_fisheye_camera(self):
        """
        获取鱼眼相机对象
        
        返回:
            FisheyeCamera: 鱼眼相机对象
        """
        if not self.is_connected:
            logger.warning("设备未连接，无法获取鱼眼相机")
            return None
        
        # 延迟导入，避免循环导入
        if self._fisheye_camera is None:
            try:
                from .camera.fisheye import FisheyeCamera
                self._fisheye_camera = FisheyeCamera(self.camera_width,self.camera_height,self.camera_fps,self.fisheye_camera_index)
                self._fisheye_camera.connect()
            except Exception as e:
                logger.error(f"初始化鱼眼相机失败: {e}")
                return None
        
        return self._fisheye_camera
    
    def get_realsense_camera(self):
        """
        获取RealSense相机对象
        
        返回:
            RealSenseCamera: RealSense相机对象
        """
        if not self.is_connected:
            logger.warning("设备未连接，无法获取RealSense相机")
            return None
        
        # 延迟导入，避免循环导入
        if self._realsense_camera is None:
            try:
                from .camera.realsense import RealSenseCamera
                self._realsense_camera = RealSenseCamera(self.camera_width,self.camera_height,self.camera_fps,self.realsense_serial_number)
                self._realsense_camera.connect()
            except Exception as e:
                logger.error(f"初始化RealSense相机失败: {e}")
                return None
        
        return self._realsense_camera
    
    def get_vive_tracker(self):
        """
        获取Vive Tracker对象
        
        返回:
            ViveTracker: Vive Tracker对象
        """
        # 延迟导入，避免循环导入
        if self._vive_tracker is None:
            try:
                from .tracker.vive_tracker import ViveTracker
                self._vive_tracker = ViveTracker(
                    config_path=self._vive_tracker_config,
                    lh_config=self._vive_tracker_lh,
                    args=self._vive_tracker_args
                )
                self._vive_tracker.connect()
            except Exception as e:
                logger.error(f"初始化Vive Tracker失败: {e}")
                return None
        
        return self._vive_tracker
    
    def get_pose(self, device_name=None):
        """
        获取Vive Tracker的位姿数据
        
        参数:
            device_name (str, optional): 设备名称，如果为None则返回所有设备的位姿数据
        
        返回:
            PoseData或dict: 如果指定了device_name，返回该设备的PoseData对象；
                          否则返回包含所有设备位姿的字典 {device_name: PoseData}
        """
        tracker = self.get_vive_tracker()
        if tracker:
            return tracker.get_pose(device_name)
        else:
            logger.warning("Vive Tracker未初始化，无法获取位姿数据")
            return None if device_name else {}
    
    def get_tracker_devices(self):
        """
        获取所有已检测到的Vive Tracker设备列表
        
        返回:
            list: 设备名称列表
        """
        tracker = self.get_vive_tracker()
        if tracker:
            return tracker.get_devices()
        else:
            logger.warning("Vive Tracker未初始化，无法获取设备列表")
            return []
    
    def __del__(self):
        """
        析构函数，确保资源被正确释放
        """
        self.disconnect()
