#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pika Gripper 设备类，提供对Pika Gripper设备的访问接口
"""

import time
import math
import logging
import threading
import struct
from .serial_comm import SerialComm

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('pika.gripper')

# 命令类型枚举
class CommandType:
    DISABLE = 10    
    ENABLE = 11
    SET_ZERO = 12
    EFFORT_CTRL = 20
    VELOCITY_CTRL = 21
    POSITION_CTRL = 22

class Gripper:
    """
    Pika Gripper设备类，提供对Pika Gripper设备的访问接口
    
    参数:
        port (str): 串口设备路径，默认为'/dev/ttyUSB0'
    """
    
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        json_pattern = r'\{\r\n"motor":\{\r\n"Speed":[^\r\n]*,\r\n"Current":[^\r\n]*,\r\n"Position":[^\r\n]*\r\n\}\r\n,\r\n"motorstatus":\{\r\n"Voltage":[^\r\n]*,\r\n"DriverTemp":[^\r\n]*,\r\n"MotorTemp":[^\r\n]*,\r\n"Status":[^\r\n]*,\r\n"BusCurrent":[^\r\n]*\r\n\}\r\n\r\n\}'
        self.serial_comm = SerialComm(port=port, json_pattern=json_pattern)
        self.is_connected = False
        self.data_lock = threading.Lock()
        self.motor_data = {
            'Speed': 0.0,  # 电机当前转速（rad/s）
            'Current': 0,    # 电机当前相电流（mA）
            'Position': 0.0  # 电机当前位置（rad）
        }
        self.motor_status = {
            'Voltage': 0.0,  # 电机驱动器电压(V)
            'DriverTemp': 0,  # 电机驱动器温度(°C)
            'MotorTemp': 0,   # 电机温度(°C)
            'Status': "0x00",  # 电机驱动器状态
            'BusCurrent': 0    # 母线电流(mA)
        }
        
        
        # 鱼眼相机索引
        self.fisheye_camera_index = 0
        
        # realsense相机序列号
        self.realsense_serial_number = None
        
        # 相机分辨率和帧率
        self.camera_width=1280
        self.camera_height=720
        self.camera_fps=30
        self.device_id=0
        
        # 相机对象，延迟初始化
        self._fisheye_camera = None
        self._realsense_camera = None
    
    def connect(self):
        """
        连接Pika Gripper设备
        
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
        logger.info(f"成功连接到Pika Gripper设备: {self.port}")
        
        # 等待初始数据
        time.sleep(0.5)
        return True
    
    def disconnect(self):
        """
        断开Pika Gripper设备连接
        """
        if not self.is_connected:
            return
        
        # 断开串口连接
        self.serial_comm.disconnect()
        self.is_connected = False
        logger.info(f"已断开Pika Gripper设备连接: {self.port}")
        
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
    
    def _data_callback(self, data):
        """
        数据回调函数，处理接收到的JSON数据
        
        参数:
            data (dict): 接收到的JSON数据
        """
        try:
            with self.data_lock:
                # 处理电机数据
                if 'motor' in data:
                    motor = data['motor']
                    self.motor_data = {
                        'Speed': motor.get('Speed', 0.0),
                        'Current': motor.get('Current', 0),
                        'Position': motor.get('Position', 0.0)
                    }
                
                # 处理电机状态
                if 'motorstatus' in data:
                    status = data['motorstatus']
                    self.motor_status = {
                        'Voltage': status.get('Voltage', 0.0),
                        'DriverTemp': status.get('DriverTemp', 0),
                        'MotorTemp': status.get('MotorTemp', 0),
                        'Status': status.get('Status', "0x00"),
                        'BusCurrent': status.get('BusCurrent', 0)
                    }
        except Exception as e:
            logger.error(f"处理数据回调异常: {e}")
    
    def get_motor_data(self):
        """
        获取电机完整数据
        
        返回:
            dict: 电机数据，包含Speed, Current, Position字段
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认电机数据")
            return {'Speed': 0.0, 'Current': 0, 'Position': 0.0}
        
        with self.data_lock:
            return self.motor_data.copy()
    
    def get_motor_status(self):
        """
        获取电机状态
        
        返回:
            dict: 电机状态，包含Voltage, DriverTemp, MotorTemp, Status, BusCurrent字段
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认电机状态")
            return {'Voltage': 0.0, 'DriverTemp': 0, 'MotorTemp': 0, 'Status': "0x00", 'BusCurrent': 0}
        
        with self.data_lock:
            return self.motor_status.copy()

    def get_motor_speed(self):
        """
        获取电机当前转速 (rad/s)
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认电机转速")
            return self.motor_data.get('Speed', 0.0)
        with self.data_lock:
            return self.motor_data['Speed']

    def get_motor_current(self):
        """
        获取电机当前相电流 (mA)
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认电机电流")
            return self.motor_data.get('Current', 0)
        with self.data_lock:
            return self.motor_data['Current']

    def get_motor_position(self):
        """
        获取电机当前位置 (rad)
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认电机位置")
            return self.motor_data.get('Position', 0.0)
        with self.data_lock:
            return self.motor_data['Position']

    def get_distance(self,angle):
        angle = (180.0 - 43.99) / 180.0 * math.pi - angle
        height = 0.0325 * math.sin(angle)
        width_d = 0.0325 * math.cos(angle)
        width = math.sqrt(0.058**2 - (height - 0.01456)**2) + width_d
        # 将单位由m转换为mm
        return width*1000
    def get_gripper_distance(self):
        """
        获取夹爪当前位置 (mm)
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认电机位置")
        with self.data_lock:
            angle = self.motor_data['Position']
            distance = (self.get_distance(angle) - self.get_distance(0)) * 2   #default
            # distance = (self.get_distance(angle) - 81.7) * 2
            return distance
            
    def get_voltage(self):
        """
        获取电机驱动器电压 (V)
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认电压")
            return self.motor_status.get('Voltage', 0.0)
        with self.data_lock:
            return self.motor_status['Voltage']

    def get_driver_temp(self):
        """
        获取电机驱动器温度 (°C)
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认驱动器温度")
            return self.motor_status.get('DriverTemp', 0)
        with self.data_lock:
            return self.motor_status['DriverTemp']

    def get_motor_temp(self):
        """
        获取电机温度 (°C)
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认电机温度")
            return self.motor_status.get('MotorTemp', 0)
        with self.data_lock:
            return self.motor_status['MotorTemp']
    
    def get_status_raw(self):
        """
        获取电机驱动器状态 (原始字符串)
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认状态")
            return self.motor_status.get('Status', "0x00")
        with self.data_lock:
            return self.motor_status['Status']

    def get_bus_current(self):
        """
        获取母线电流 (mA)
        """
        if not self.is_connected:
            logger.warning("设备未连接，返回默认母线电流")
            return self.motor_status.get('BusCurrent', 0)
        with self.data_lock:
            return self.motor_status['BusCurrent']
    
    def enable(self):
        """
        启用电机
        
        返回:
            bool: 操作是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法启用电机")
            return False
        
        return self.serial_comm.send_command(CommandType.ENABLE)
    
    def disable(self):
        """
        禁用电机
        
        返回:
            bool: 操作是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法禁用电机")
            return False
        
        return self.serial_comm.send_command(CommandType.DISABLE)
    
    def set_zero(self):
        """
        设置零点
        
        返回:
            bool: 操作是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法设置零点")
            return False
        
        return self.serial_comm.send_command(CommandType.SET_ZERO)
    
    def set_motor_angle(self, rad):
        """
        设置电机转动弧度
        
        参数:
            rad (float): 目标弧度，单位为弧度
            
        返回:
            bool: 操作是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法设置电机弧度")
            return False
        
        # 确保角度非负
        if rad < 0:
            rad = 0
            logger.warning("电机弧度不能为负值，已设置为0")
        
        return self.serial_comm.send_command(CommandType.POSITION_CTRL, rad)
    
    def set_gripper_distance(self, target_gripper_distance_mm):
        """
        设置夹爪开合距离 (mm)
        
        参数:
            target_gripper_distance_mm (float): 目标夹爪开合距离 (mm)
            取值范围：0-90mm
        返回:
            bool: 操作是否成功
        """
        if not self.is_connected:
            logger.warning("设备未连接，无法设置夹爪距离")
            return False

        # 1. 根据目标夹爪行程距离反推出 get_distance(angle) 的目标值
        # get_gripper_distance = (get_distance(angle) - get_distance(0)) * 2
        # target_gripper_distance_mm / 2 = get_distance(angle) - get_distance(0)
        # get_distance(angle) = target_gripper_distance_mm / 2 + get_distance(0)
        
        # Calculate get_distance(0) once
        get_distance_0 = self.get_distance(0) 
        target_width_mm = target_gripper_distance_mm / 2 + get_distance_0

        # 2. 通过数值方法反向查找对应的电机角度
        # 我们需要找到一个 'angle'，使得 self.get_distance(angle) 尽可能接近 target_width_mm
        
        # 定义搜索范围和精度
        # 假设电机角度在 0 到 (180.0 - 43.99) / 180.0 * math.pi 弧度之间
        low_angle = 0.0
        high_angle = (180.0 - 43.99) / 180.0 * math.pi # Approximately 1.99 radians
        
        # 增加一个检查，确保目标距离在夹爪的有效范围内
        # 最小夹爪距离 (angle = low_angle)
        min_gripper_distance_at_low_angle = (self.get_distance(low_angle) - get_distance_0) * 2
        # 最大夹爪距离 (angle = high_angle)
        max_gripper_distance_at_high_angle = (self.get_distance(high_angle) - get_distance_0) * 2

        # Ensure target_gripper_distance_mm is within the valid range
        if not (min_gripper_distance_at_low_angle <= target_gripper_distance_mm <= max_gripper_distance_at_high_angle):
            logger.error(f"目标夹爪距离 {target_gripper_distance_mm:.2f} mm 超出有效范围 [{min_gripper_distance_at_low_angle:.2f}, {max_gripper_distance_at_high_angle:.2f}] mm")
            return False

        tolerance = 0.01 # 夹爪距离的容差 (mm)
        angle_tolerance = 0.00001 # 角度的容差 (rad)
        max_iterations = 1000

        found_angle = None

        for i in range(max_iterations):
            mid_angle = (low_angle + high_angle) / 2
            current_width_mm = self.get_distance(mid_angle)
            
            # Check if the current_width_mm is close enough to target_width_mm
            if abs(current_width_mm - target_width_mm) < tolerance:
                found_angle = mid_angle
                break
            
            # Since get_distance(angle) increases with angle:
            if current_width_mm < target_width_mm:
                low_angle = mid_angle
            else:
                high_angle = mid_angle
            
            # If the angle range becomes too small, stop
            if (high_angle - low_angle) < angle_tolerance:
                found_angle = mid_angle # Use the midpoint as the best approximation
                break

        if found_angle is not None:
            self.set_motor_angle(found_angle)
            logger.info(f"夹爪已设置为目标距离 {target_gripper_distance_mm} mm，对应电机角度 {found_angle:.4f} rad")
            return True
        else:
            logger.error(f"未能找到目标夹爪距离 {target_gripper_distance_mm} mm 对应的电机角度")
            return False
    def set_velocity(self, velocity):
        """
        设置电机速度
        
        参数:
            velocity (float): 目标速度
            
        返回:
            bool: 操作是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法设置速度")
            return False
        
        return self.serial_comm.send_command(CommandType.VELOCITY_CTRL, velocity)
    
    def set_effort(self, effort):
        """
        设置电机力矩
        
        参数:
            effort (float): 目标力矩
            
        返回:
            bool: 操作是否成功
        """
        if not self.is_connected:
            logger.error("设备未连接，无法设置力矩")
            return False
        
        return self.serial_comm.send_command(CommandType.EFFORT_CTRL, effort)
    
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
    
    def __del__(self):
        """
        析构函数，确保资源被正确释放
        """
        self.disconnect()

