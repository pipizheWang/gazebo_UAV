#!/usr/bin/env python3
import rclpy
import numpy as np
from numpy.linalg import norm
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
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
        log_filename = f"trajectory_log_{uav_id}_{current_time}.csv"

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
                                  "Position_Error", "Traj_time", "Yaw"])

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

        # 发布器 - 同样去掉开头的斜杠
        self.position_setpoint_pub_ = self.create_publisher(
            PositionTarget, 'mavros/setpoint_raw/local', qos_reliable)

        self.target_pose_pub_ = self.create_publisher(
            PoseStamped, 'trajectory/target_pose', qos_reliable)

        self.controller_timer_ = self.create_timer(1 / self.control_rate, self.controller_cb)

        # 初始化参数
        self.declare_parameter('traj_mode', False)  # 轨迹模式开关
        
        # 新增：不同高度偏移参数（用于上下飞行）
        self.declare_parameter('height_offset', 0.0)  # 高度偏移量

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

    def controller_cb(self):
        """控制器主回调函数"""

        if self.current_pa is None or self.current_velo is None:
            self.get_logger().warn("Waiting for pose and velocity data...")
            return

        # 获取参数
        traj_mode = self.get_parameter('traj_mode').value
        height_offset = self.get_parameter('height_offset').value

        # 更新轨迹时间
        self.update_trajectory_time(traj_mode)

        # 获取当前位置和速度
        current_pose = np.array([
            [self.current_pa.pose.position.x],
            [self.current_pa.pose.position.y],
            [self.current_pa.pose.position.z]
        ])

        current_velo = np.array([
            [self.current_velo.twist.linear.x],
            [self.current_velo.twist.linear.y],
            [self.current_velo.twist.linear.z]
        ])

        timestamp = self.clock.now().nanoseconds * 1e-9

        if traj_mode and self.traj_t >= 0.0:
            # 获取目标轨迹
            traj_p = self.traj.pose(self.traj_t)
            traj_yaw = self.traj.yaw(self.traj_t)
            
            # 应用高度偏移（用于上下飞行配置）
            traj_p[2, 0] += height_offset

            # 计算位置误差
            position_error = norm(current_pose - traj_p)

            # 记录数据（含目标轨迹信息）
            self.csv_writer.writerow([
                timestamp,
                current_pose[0, 0], current_pose[1, 0], current_pose[2, 0],
                current_velo[0, 0], current_velo[1, 0], current_velo[2, 0],
                traj_p[0, 0], traj_p[1, 0], traj_p[2, 0],
                position_error, self.traj_t, traj_yaw
            ])

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

            # 创建并发布位置指令
            position_target = PositionTarget()
            position_target.header.stamp = self.get_clock().now().to_msg()
            position_target.header.frame_id = "map"

            # 设置坐标系为LOCAL_NED
            position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

            # 设置类型掩码：启用位置、速度和偏航角控制
            position_target.type_mask = (
                    PositionTarget.IGNORE_AFX |
                    PositionTarget.IGNORE_AFY |
                    PositionTarget.IGNORE_AFZ |
                    PositionTarget.FORCE
            )

            # 设置目标位置
            position_target.position.x = float(traj_p[0][0])
            position_target.position.y = float(traj_p[1][0])
            position_target.position.z = float(traj_p[2][0])

            # 设置目标偏航角
            position_target.yaw = float(traj_yaw)

            # 发布位置指令
            self.position_setpoint_pub_.publish(position_target)
        else:
            # 不在轨迹模式下，只记录当前状态
            self.csv_writer.writerow([
                timestamp,
                current_pose[0, 0], current_pose[1, 0], current_pose[2, 0],
                current_velo[0, 0], current_velo[1, 0], current_velo[2, 0],
                0.0, 0.0, 0.0,  # 目标位置为零
                0.0, -1.0, 0.0  # 误差、轨迹时间和偏航角为零
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
        node = TrajController("traj_controller_position")
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