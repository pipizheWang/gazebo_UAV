import numpy as np

class TargetTraj():
    def __init__(self, FLAG = 1, h_default = 5.0):
        self.FLAG = FLAG    #选择轨迹类型，1：xy平面圆轨迹 2:平面8字（里萨如图形）
        self.R = 6.0        #圆轨迹半径
        self.w = (2*np.pi)/30#轨迹周期
        self.h_default = h_default

    #若t<0,视为尚未开始跟踪轨迹
    def pose(self, t):#目标位置
        if self.FLAG == 1 and t >= 0:
            x = self.R * np.sin(self.w*t)
            y = self.R * (1 - np.cos(self.w*t))
            z = self.h_default
            return np.array([[x], [y], [z]])
        elif self.FLAG == 2 and t >= 0:
            x = self.R * np.sin(2*self.w*t)
            y = 2*self.R * (1 - np.cos(self.w*t))
            z = self.h_default
            return np.array([[x], [y], [z]])
        elif self.FLAG == 3 and t >= 0:
            return np.array([[0.0], [0.0], [self.h_default]])
        elif t < 0:
            return np.array([[0.0], [0.0], [self.h_default]])
        
    def velo(self, t):#目标速度
        if self.FLAG == 1 and t >= 0:
            vx = self.R * self.w * np.cos(self.w*t)
            vy = self.R * self.w * np.sin(self.w*t)
            vz = 0.0
            return np.array([[vx], [vy], [vz]])
        elif self.FLAG == 2 and t >= 0:
            vx = self.R * 2*self.w * np.cos(2*self.w*t)
            vy = 2*self.R * self.w * np.sin(self.w*t)
            vz = 0.0
            return np.array([[vx], [vy], [vz]])
        elif self.FLAG == 3 and t >= 0:
            return np.array([[0.0], [0.0], [0.0]])
        elif t < 0:
            return np.array([[0.0], [0.0], [0.0]])
        
    def acce(self, t):#目标加速度
        if self.FLAG == 1 and t >= 0:
            ax = -self.R * self.w * self.w * np.sin(self.w*t)
            ay = self.R * self.w * self.w * np.cos(self.w*t)
            az = 0.0
            return np.array([[ax], [ay], [az]])
        elif self.FLAG == 2 and t >= 0:
            ax = -self.R * 2*self.w * 2*self.w * np.sin(2*self.w*t)
            ay = 2*self.R * self.w * self.w * np.cos(self.w*t)
            az = 0.0
            return np.array([[ax], [ay], [az]])
        elif self.FLAG == 3 and t >= 0:
            return np.array([[0.0], [0.0], [0.0]])
        elif t < 0:
            return np.array([[0.0], [0.0], [0.0]])
        
    def yaw (self, t):#给定偏航角
        if self.FLAG == 1 and t >= 0:
            yaw = 0.0
            return yaw
        elif self.FLAG == 2 and t >= 0:
            yaw = 0.0
            return yaw
        elif self.FLAG == 3 and t >= 0:
            yaw = 0.0
            return yaw
        elif t < 0:
            return 0.0