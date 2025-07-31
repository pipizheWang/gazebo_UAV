#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from mavros_msgs.msg import State, AttitudeTarget, Thrust
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
import csv
import os
from datetime import datetime


def vee(antisym_mat):
    """从反对称矩阵提取向量"""
    return np.array([
        antisym_mat[2, 1],
        antisym_mat[0, 2],
        antisym_mat[1, 0]
    ])


class AttitudeController(Node):
    def __init__(self, name):
        # 初始化
        super().__init__(name)
        self.get_logger().info("Node running: %s" % name)

        # 获取命名空间，用于区分不同无人机
        self.namespace = self.get_namespace()
        if self.namespace == '/':
            self.namespace = ''
        
        # 去掉命名空间开头的斜杠（如果有的话）
        uav_id = self.namespace.strip('/')
        if not uav_id:
            uav_id = "default"
        
        self.get_logger().info(f"UAV ID: {uav_id}, Namespace: {self.namespace}")

        # 使用当前时间和UAV ID创建唯一的文件名
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_filename = f"attitude_control_log_{uav_id}_{current_time}.csv"

        # 确保日志目录存在
        log_dir = "/home/swarm/wz/Log"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # 初始化数据记录
        self.log_file_path = os.path.join(log_dir, log_filename)
        self.log_file = open(self.log_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["Time", "Current_Roll", "Current_Pitch", "Current_Yaw",
                                  "Desired_Roll", "Desired_Pitch", "Desired_Yaw",
                                  "Error_Roll", "Error_Pitch", "Error_Yaw", 
                                  "Omega_x", "Omega_y", "Omega_z", "Thrust"])

        self.get_logger().info(f"Attitude logging data to {self.log_file_path}")

        # 控制频率
        self.Rate = 200.0

        # 初始化状态变量
        self.current_pa = None
        self.current_velo = None
        self.current_cmd = None

        # 订阅和发布 - 重点修改：使用相对路径支持命名空间
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 订阅器 - 去掉开头的斜杠，让ROS2自动处理命名空间
        self.pa_sub_ = self.create_subscription(
            PoseStamped, 'mavros/local_position/pose', self.pa_cb, qos_best_effort)
        self.velo_sub_ = self.create_subscription(
            TwistStamped, 'mavros/local_position/velocity_local', self.velo_cb, qos_best_effort)
        
        # 订阅中间话题（来自上层轨迹控制器）
        self.atti_sub_ = self.create_subscription(
            AttitudeTarget, 'control/attitude', self.atti_cb, qos_best_effort)

        # 发布到MAVROS
        self.controller_pub_ = self.create_publisher(
            AttitudeTarget, 'mavros/setpoint_raw/attitude', qos_reliable)

        self.controller_timer_ = self.create_timer((1 / self.Rate), self.controller_cb)

        # 初始化控制系数
        self.declare_parameter('P_gain', [8.0, 8.0, 2.0])

    def pa_cb(self, msg):
        self.current_pa = msg

    def velo_cb(self, msg):
        self.current_velo = msg

    def atti_cb(self, msg):
        self.current_cmd = msg

    def get_current_state(self):
        """获取当前状态和期望状态"""
        if self.current_pa is None or self.current_cmd is None:
            return None, None
        
        # 当前姿态四元数
        quaternion = [
            self.current_pa.pose.orientation.x,
            self.current_pa.pose.orientation.y,
            self.current_pa.pose.orientation.z,
            self.current_pa.pose.orientation.w
        ]
        DCM = R.from_quat(quaternion).as_matrix()

        # 期望姿态四元数
        quaternion_sp = [
            self.current_cmd.orientation.x,
            self.current_cmd.orientation.y,
            self.current_cmd.orientation.z,
            self.current_cmd.orientation.w
        ]
        DCM_sp = R.from_quat(quaternion_sp).as_matrix()

        return DCM, DCM_sp

    def calculate_desired_omega(self, DCM, DCM_sp, P_gain):
        """计算期望角速度"""
        if DCM is None or DCM_sp is None:
            return np.zeros(3)

        # 计算姿态误差
        e_DCM = 0.5 * vee(DCM_sp.T @ DCM - DCM.T @ DCM_sp)
        
        # 计算期望角速度
        omega_d = -DCM.T @ DCM_sp @ P_gain @ e_DCM
        
        return omega_d

    def log_attitude_data(self, DCM, DCM_sp, omega_d, thrust):
        """记录姿态控制数据"""
        # 当前欧拉角
        current_euler = R.from_matrix(DCM).as_euler('xyz')
        # 期望欧拉角
        desired_euler = R.from_matrix(DCM_sp).as_euler('xyz')
        # 误差
        error_euler = current_euler - desired_euler
        
        # 处理角度环绕问题
        for i in range(3):
            while error_euler[i] > np.pi:
                error_euler[i] -= 2 * np.pi
            while error_euler[i] < -np.pi:
                error_euler[i] += 2 * np.pi
        
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        
        self.csv_writer.writerow([
            timestamp,
            current_euler[0], current_euler[1], current_euler[2],  # 当前RPY
            desired_euler[0], desired_euler[1], desired_euler[2],  # 期望RPY
            error_euler[0], error_euler[1], error_euler[2],        # 误差RPY
            omega_d[0], omega_d[1], omega_d[2],                    # 期望角速度
            thrust                                                  # 推力
        ])
        
        # 确保数据实时写入文件
        self.log_file.flush()

    def controller_cb(self):
        """姿态控制器主回调函数"""
        if self.current_pa is None or self.current_cmd is None:
            # 静默等待，不要频繁打印警告
            return

        # 获取控制增益参数
        P_gain = np.diag(self.get_parameter('P_gain').value)
        
        # 获取当前状态和期望状态
        DCM, DCM_sp = self.get_current_state()
        
        if DCM is None or DCM_sp is None:
            return
        
        # 计算期望角速度
        omega_d = self.calculate_desired_omega(DCM, DCM_sp, P_gain)

        # 创建MAVROS姿态控制指令
        attitude_target = AttitudeTarget()
        attitude_target.header.stamp = self.get_clock().now().to_msg()
        attitude_target.header.frame_id = "map"
        
        # 从上层控制器获取推力
        attitude_target.thrust = self.current_cmd.thrust
        
        # 设置期望角速度
        attitude_target.body_rate.x = float(omega_d[0])
        attitude_target.body_rate.y = float(omega_d[1])
        attitude_target.body_rate.z = float(omega_d[2])
        
        # 设置控制掩码：只控制角速度和推力
        attitude_target.type_mask = int(128)  # 忽略姿态，只控制角速度

        # 发布控制指令
        self.controller_pub_.publish(attitude_target)

        # 记录数据
        self.log_attitude_data(DCM, DCM_sp, omega_d, attitude_target.thrust)

        # 调试信息（降低频率）
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 50 == 0:  # 每50次循环打印一次（约4Hz）
            error_norm = np.linalg.norm(0.5 * vee(DCM_sp.T @ DCM - DCM.T @ DCM_sp))
            self.get_logger().debug(f"Attitude error norm: {error_norm:.4f}, Thrust: {attitude_target.thrust:.3f}")

    def __del__(self):
        """析构函数，确保文件被正确关闭"""
        if hasattr(self, 'log_file') and self.log_file is not None:
            self.log_file.close()


def main(args=None):
    try:
        rclpy.init(args=args)
        node = AttitudeController("attitude_controller")
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        # 确保关闭文件
        if 'node' in locals() and hasattr(node, 'log_file'):
            node.log_file.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()