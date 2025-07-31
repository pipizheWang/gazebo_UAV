#!/usr/bin/env python3
import rclpy
import numpy as np
from numpy.linalg import norm
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy
from .traj import TargetTraj
from rclpy.clock import Clock
import csv
import os
from datetime import datetime


class TrajController(Node):
    def __init__(self, name):
        # 初始化
        super().__init__(name)
        self.get_logger().info("Node running: %s" % name)

        # 控制频率
        self.control_rate = 50.0
        self.traj = TargetTraj(FLAG=3)

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
        log_filename = f"sliding_control_log_{uav_id}_{current_time}.csv"

        # 确保日志目录存在
        log_dir = "/home/swarm/wz/Log"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # 初始化数据记录
        self.log_file_path = os.path.join(log_dir, log_filename)
        self.log_file = open(self.log_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["Time", "Pos_x", "Pos_y", "Pos_z",
                                  "Vel_x", "Vel_y", "Vel_z",
                                  "Des_x", "Des_y", "Des_z",
                                  "Des_vx", "Des_vy", "Des_vz",
                                  "Force_x", "Force_y", "Force_z",
                                  "Thrust", "Position_Error", "Traj_time"])

        self.get_logger().info(f"Logging data to {self.log_file_path}")

        # 初始化时钟
        self.clock = Clock()

        # 初始化状态变量
        self.current_pa = None
        self.current_velo = None

        # 轨迹计时器
        self.traj_t = -1.0
        self.t_0 = self.clock.now()

        # 订阅和发布 - 重点修改：使用相对路径支持命名空间
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 订阅器 - 去掉开头的斜杠，让ROS2自动处理命名空间
        self.pa_sub_ = self.create_subscription(
            PoseStamped, 'mavros/local_position/pose', self.pa_cb, qos_best_effort)
        self.velo_sub_ = self.create_subscription(
            TwistStamped, 'mavros/local_position/velocity_local', self.velo_cb, qos_best_effort)

        # 发布器 - 输出到中间话题（相对路径，支持命名空间）
        self.controller_pub_ = self.create_publisher(
            AttitudeTarget, 'control/attitude', qos_reliable)

        # 可视化发布器
        self.target_pose_pub_ = self.create_publisher(
            PoseStamped, 'trajectory/target_pose', qos_reliable)

        self.controller_timer_ = self.create_timer(1 / self.control_rate, self.controller_cb)

        # 初始化参数
        self.declare_parameter('sliding_gain', [0.3, 0.3, 0.5])  # 滑模跟踪增益
        self.declare_parameter('tracking_gain', [3.0, 3.0, 5.0])  # 跟踪增益
        self.declare_parameter('traj_mode', False)  # 轨迹模式开关
        
        # 新增：不同高度偏移参数（用于上下飞行）
        self.declare_parameter('height_offset', 0.0)  # 高度偏移量

        # 系统常量
        self.gravity = 9.8
        self.thrust_efficiency = 0.74

    def pa_cb(self, msg):
        self.current_pa = msg

    def velo_cb(self, msg):
        self.current_velo = msg

    def update_trajectory_time(self, traj_mode):
        if traj_mode and (self.traj_t is None or self.traj_t == -1.0):
            self.t_0 = self.clock.now()
            self.traj_t = 0.0
            self.get_logger().info("Starting trajectory tracking")
        elif traj_mode:
            self.traj_t = (self.clock.now() - self.t_0).nanoseconds * 1e-9
        else:
            self.traj_t = -1.0
            self.t_0 = self.clock.now()

    def get_current_state(self):
        """获取当前状态"""
        # 当前位置
        pose = np.array([
            [self.current_pa.pose.position.x],
            [self.current_pa.pose.position.y],
            [self.current_pa.pose.position.z]
        ])

        # 当前速度
        velo = np.array([
            [self.current_velo.twist.linear.x],
            [self.current_velo.twist.linear.y],
            [self.current_velo.twist.linear.z]
        ])

        # 当前姿态四元数
        quaternion = [
            self.current_pa.pose.orientation.x,
            self.current_pa.pose.orientation.y,
            self.current_pa.pose.orientation.z,
            self.current_pa.pose.orientation.w
        ]

        # 计算旋转矩阵（body坐标系到world坐标系）
        r = R.from_quat(quaternion)
        rotation_matrix = r.as_matrix()

        # body系z轴方向在world坐标系中的表示
        body_z = np.dot(rotation_matrix, np.array([[0], [0], [1]]))

        return pose, velo, rotation_matrix, body_z

    def calculate_desired_force(self, pose, velo, traj_p, traj_v, traj_a, sliding_gain, tracking_gain):
        """计算期望力"""
        # 计算复合误差
        s = (velo - traj_v + sliding_gain * (pose - traj_p))

        # 前馈参考加速度
        a_r = traj_a - sliding_gain * (velo - traj_v) + np.array([[0], [0], [self.gravity]])

        # 计算期望力（包含重力补偿、跟踪控制和自适应补偿）
        F_sp = a_r - tracking_gain * s

        # 限制输出范围
        F_sp = np.clip(F_sp, np.array([[-5.0], [-5.0], [0.0]]), np.array([[5.0], [5.0], [19.6]]))

        return F_sp

    def calculate_attitude_from_force(self, F_sp, body_z, yaw_sp):
        """从期望力计算期望姿态"""
        # 计算所需推力大小（点乘）
        thrust = float(np.dot(F_sp.T, body_z))

        # 创建控制消息 - 注意：这里输出到中间话题
        attitude_target = AttitudeTarget()
        attitude_target.header.stamp = self.get_clock().now().to_msg()
        attitude_target.header.frame_id = "map"

        # 计算归一化推力值（考虑效率）
        normalized_thrust = np.sqrt(thrust / self.gravity * self.thrust_efficiency * self.thrust_efficiency)
        attitude_target.thrust = np.clip(normalized_thrust, 0.0, 1.0)

        # 基于期望力方向计算期望姿态
        if norm(F_sp) > 1e-6:  # 避免除零错误
            body_z_sp = F_sp / norm(F_sp)  # 期望z轴方向
        else:
            body_z_sp = np.array([[0], [0], [1]])  # 默认向上

        # 使用期望偏航角创建x参考方向
        x_C = np.array([[np.cos(yaw_sp)], [np.sin(yaw_sp)], [0]])

        # 计算期望y轴
        cross_product = np.cross(body_z_sp.flatten(), x_C.flatten())
        if norm(cross_product) > 1e-6:
            body_y_sp = cross_product.reshape(3, 1) / norm(cross_product)
        else:
            body_y_sp = np.array([[0], [1], [0]])  # 默认y方向

        # 计算期望x轴
        body_x_sp = np.cross(body_y_sp.flatten(), body_z_sp.flatten()).reshape(3, 1)

        # 构建期望旋转矩阵
        RM_sp = np.hstack([body_x_sp, body_y_sp, body_z_sp])

        # 转换为四元数
        r_sp = R.from_matrix(RM_sp)
        quaternion_sp = r_sp.as_quat()

        # 设置期望姿态
        attitude_target.orientation.x = quaternion_sp[0]
        attitude_target.orientation.y = quaternion_sp[1]
        attitude_target.orientation.z = quaternion_sp[2]
        attitude_target.orientation.w = quaternion_sp[3]

        # 注意：这里不设置type_mask，让下层姿态控制器处理
        return attitude_target

    def controller_cb(self):
        """控制器主回调函数"""

        if self.current_pa is None or self.current_velo is None:
            self.get_logger().warn("Waiting for pose and velocity data...")
            return

        # 获取参数
        sliding_gain = np.array(self.get_parameter('sliding_gain').value).reshape(3, 1)
        tracking_gain = np.array(self.get_parameter('tracking_gain').value).reshape(3, 1)
        traj_mode = self.get_parameter('traj_mode').value
        height_offset = self.get_parameter('height_offset').value

        # 更新轨迹时间
        self.update_trajectory_time(traj_mode)

        timestamp = self.clock.now().nanoseconds * 1e-9

        if traj_mode and self.traj_t >= 0.0:
            # 获取目标轨迹
            traj_p = self.traj.pose(self.traj_t)
            traj_v = self.traj.velo(self.traj_t)
            traj_a = self.traj.acce(self.traj_t)
            traj_yaw = self.traj.yaw(self.traj_t)
            
            # 应用高度偏移（用于上下飞行配置）
            traj_p[2, 0] += height_offset
            # 注意：速度和加速度不需要偏移，因为偏移是常数

            # 获取当前状态
            pose, velo, rotation_matrix, body_z = self.get_current_state()

            # 计算期望力
            F_sp = self.calculate_desired_force(
                pose, velo, traj_p, traj_v, traj_a, sliding_gain, tracking_gain)

            # 计算期望姿态和推力，输出到中间话题
            attitude_target = self.calculate_attitude_from_force(F_sp, body_z, traj_yaw)

            # 发布到中间话题（让姿态控制器订阅）
            self.controller_pub_.publish(attitude_target)

            # 创建并发布目标姿态（用于可视化）
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = float(traj_p[0][0])
            target_pose.pose.position.y = float(traj_p[1][0])
            target_pose.pose.position.z = float(traj_p[2][0])

            # 创建目标轨迹的四元数（基于偏航角）
            r = R.from_euler('z', traj_yaw)
            quat = r.as_quat()
            target_pose.pose.orientation.x = quat[0]
            target_pose.pose.orientation.y = quat[1]
            target_pose.pose.orientation.z = quat[2]
            target_pose.pose.orientation.w = quat[3]

            self.target_pose_pub_.publish(target_pose)

            # 计算位置误差
            position_error = norm(pose - traj_p)

            # 记录详细数据
            self.csv_writer.writerow([
                timestamp,
                pose[0, 0], pose[1, 0], pose[2, 0],
                velo[0, 0], velo[1, 0], velo[2, 0],
                traj_p[0, 0], traj_p[1, 0], traj_p[2, 0],
                traj_v[0, 0], traj_v[1, 0], traj_v[2, 0],
                F_sp[0, 0], F_sp[1, 0], F_sp[2, 0],
                attitude_target.thrust, position_error, self.traj_t
            ])

            # 记录调试信息
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                self.get_logger().debug(f"Traj time: {self.traj_t:.2f}, Thrust: {attitude_target.thrust:.3f}")
                self.get_logger().debug(f"Position error: {position_error:.3f}m")
        else:
            # 不在轨迹模式下，发布悬停指令
            pose, velo, rotation_matrix, body_z = self.get_current_state()
            
            # 创建悬停姿态指令
            attitude_target = AttitudeTarget()
            attitude_target.header.stamp = self.get_clock().now().to_msg()
            attitude_target.header.frame_id = "map"
            
            # 基本悬停推力（补偿重力）
            attitude_target.thrust = 0.5  # 大约对应重力补偿
            
            # 保持当前姿态
            attitude_target.orientation = self.current_pa.pose.orientation
            
            # 发布到中间话题
            self.controller_pub_.publish(attitude_target)

            # 记录悬停状态数据
            self.csv_writer.writerow([
                timestamp,
                pose[0, 0], pose[1, 0], pose[2, 0],
                velo[0, 0], velo[1, 0], velo[2, 0],
                0.0, 0.0, 0.0,  # 目标位置为零
                0.0, 0.0, 0.0,  # 目标速度为零
                0.0, 0.0, 0.0,  # 期望力为零  
                0.5, 0.0, -1.0  # 悬停推力、误差、轨迹时间
            ])

        # 确保数据实时写入文件
        self.log_file.flush()

    def __del__(self):
        """析构函数，确保文件被正确关闭"""
        if hasattr(self, 'log_file') and self.log_file is not None:
            self.log_file.close()


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TrajController("traj_controller_NL")
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