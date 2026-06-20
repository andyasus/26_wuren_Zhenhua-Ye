"""
扩展卡尔曼滤波（EKF）核心算法
用于融合GPS、IMU、磁力计、里程计数据，估计车辆位姿

状态向量（5维）：
  [px, py, yaw, v, yaw_rate]
  px, py    : world坐标系下的位置（米）
  yaw       : 航向角（弧度），0=东，pi/2=北
  v         : 前向速度（米/秒）
  yaw_rate  : 航向角速度（弧度/秒）

传感器观测模型：
  GPS       : 观测 px, py
  磁力计     : 观测 yaw
  里程计     : 观测 v
  IMU       : 观测 yaw_rate
"""

import numpy as np

class EKF:
    """扩展卡尔曼滤波器"""

    def __init__(self):
        """初始化状态向量和协方差矩阵"""
        # 状态向量 [px,py,yaw,v,yaw_rate]
        self.x = np.zeros(5)

        # 状态协方差矩阵 P （表示对状态估计的不确定性）
        self.P = np.eye(5) * 0.1

        # 过程噪声协方差 Q （模型不确定性）
        # 数值越大，越相信传感器，越不相信模型
        self.Q = np.diag([0.1,0.1,0.05,0.5,0.1])

        # 观测噪声协方差 R （传感器噪声）
        # 数值越大，越不相信该传感器
        # GPS的噪声（位置测量，米）
        self.R_gps = np.diag([0.5,0.5])

        # 磁力计的噪声（航向角测量，弧度）
        self.R_mag = np.diag([0.05])

        # 里程计的噪声（速度测量，米/秒）
        self.R_odo = np.diag([0.1])

        # IMU角速度的噪声（弧度/秒）
        self.R_imu = np.diag([0.05])

    def normalize_angle(self, angle):
        """将角度归一化到[-pi,pi]区间"""
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle
    
    def predict(self, dt):
        """
        根据运动模型预测下一时刻的状态
        dt：时间步长（秒）
        """
        # 1. 状态预测
        px, py, yaw, v, yaw_rate = self.x
        self.x[0] = px + v * np.cos(yaw) * dt
        self.x[1] = py + v * np.sin(yaw) * dt
        self.x[2] = yaw + yaw_rate * dt
        self.x[3] = v
        self.x[4] = yaw_rate

        # 归一化航向角
        self.x[2] = self.normalize_angle(self.x[2])

        # 2. 协方差预测（雅可比矩阵 F）
        F = np.eye(5)
        F[0,2] = -v * np.sin(yaw) * dt # px受yaw的影响
        F[0,3] = np.cos(yaw) * dt # px受v的影响
        F[1,2] = v * np.cos(yaw) * dt # py受yaw的影响
        F[1,3] = np.sin(yaw) * dt # py受v的影响
        F[2,4] = dt # yaw受yaw_rate的影响

        # 协方差更新：P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q # @点积 .T转置

    def _update(self, H, z, R):
        """
        通用卡尔曼滤波更新步骤
        H：观测矩阵（将5维状态映射到观测空间）
        z：实际观测值（numpy数组）
        R：观测噪声协方差矩阵
        """
        # 残差：实际 - 预测
        y = z - H @ self.x

        # 卡尔曼增益：K = P * H^T * (H * P * H^T + R)^(-1)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # 状态更新
        self.x = self.x + K @ y

        # 协方差更新
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_gps(self, gps_x, gps_y):
        """用GPS更新位置"""
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])
        z = np.array([gps_x, gps_y])
        self._update(H, z, self.R_gps)

    def update_magnetometer(self, yaw):
        """用磁力计更新航向角"""
        H = np.array([[0,0,1,0,0]]) # 只观测yaw
        z = np.array([yaw])
        self._update(H, z, self.R_mag)

    def update_odometry(self, v):
        """用里程计更新速度"""
        H = np.array([[0,0,0,1,0]])
        z = np.array([v])
        self._update(H, z, self.R_odo)

    def update_imu_yaw_rate(self, yaw_rate):
        """用IMU更新航向角速度"""
        H = np.array([[0,0,0,0,1]])
        z = np.array([yaw_rate])
        self._update(H, z, self.R_imu)

    def get_state(self):
        """获取当前状态估计"""
        return self.x.copy()

    def get_position(self):
        """获取位置 (px, py)"""
        return self.x[0], self.x[1]

    def get_yaw(self):
        """获取航向角"""
        return self.x[2]

