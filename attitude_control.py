#!usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from mavros_msgs.msg import State, AttitudeTarget, Thrust
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def vee(antisym_mat):
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

        # 控制频率
        self.Rate = 200.0

        # 初始化状态变量
        self.current_pa = None
        self.current_velo = None
        self.current_cmd = None

        # 订阅和发布
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pa_sub_ = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pa_cb, qos_best_effort)
        self.velo_sub_ = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.velo_cb, qos_best_effort)
        self.atti_sub_ = self.create_subscription(
            AttitudeTarget,'/control/attitude', self.atti_cb, qos_best_effort)

        self.controller_pub_ = self.create_publisher(
            AttitudeTarget, '/mavros/setpoint_raw/attitude',qos_reliable)

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
        quaternion = [
            self.current_pa.pose.orientation.x,
            self.current_pa.pose.orientation.y,
            self.current_pa.pose.orientation.z,
            self.current_pa.pose.orientation.w
        ]
        DCM = R.from_quat(quaternion).as_matrix()

        quaternion_sp = [
            self.current_cmd.orientation.x,
            self.current_cmd.orientation.y,
            self.current_cmd.orientation.z,
            self.current_cmd.orientation.w
        ]
        DCM_sp = R.from_quat(quaternion_sp).as_matrix()

        return DCM, DCM_sp

    def calculate_desired_omega(self, DCM, DCM_sp, P_gain):
        if self.current_pa is None or self.current_cmd is None:
            self.get_logger().warn("Waiting for pose and attitude command data...")
            return np.zeros(3)

        e_DCM = 0.5 * vee(DCM_sp.T @ DCM - DCM.T @ DCM_sp)
        omega_d = -DCM.T @ DCM_sp @ P_gain @ e_DCM
        return omega_d

    def controller_cb(self):
        if self.current_pa is None or self.current_cmd is None:
            self.get_logger().warn("Waiting for pose and attitude command data...")
            return

        P_gain = np.diag(self.get_parameter('P_gain').value)
        DCM, DCM_sp = self.get_current_state()
        omega_d = self.calculate_desired_omega(DCM, DCM_sp, P_gain)

        attitude_target = AttitudeTarget()
        attitude_target.thrust = self.current_cmd.thrust
        attitude_target.body_rate.x = omega_d[0]
        attitude_target.body_rate.y = omega_d[1]
        attitude_target.body_rate.z = omega_d[2]
        attitude_target.type_mask = int(128)

        self.controller_pub_.publish(attitude_target)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = AttitudeController("attitude_controller")
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()