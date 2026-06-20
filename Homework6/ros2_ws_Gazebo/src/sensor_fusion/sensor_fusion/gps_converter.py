"""
GPS转换模块
功能：将GPS经纬度数据转换为world坐标系下的米制坐标(x,y)
坐标系说明：
world坐标系：东北天
使用第一帧GPS数据作为原点，后续数据相对于原点计算偏移

参考：
    局部切平面：将经纬度差投影到切平面上，近似为米制距离
    纬度1度约等于111320米，经度1度约等于111320*cos(lat)米
"""

import numpy as np
import math

class GPSConverter:
    EARTH_RADIUS = 6371000.0 # 地球平均半径/m

    def __init__(self):
        self.origin_lat = None # 原点纬度（弧度）
        self.origin_lon = None # 原点经度（弧度）
        self.is_initialized = False # 是否已用第一帧数据初始化
        # 车辆起始位置（world坐标系下）
        self.vehicle_start_x = 0.0
        self.vehicle_start_y = -15.0
        
    def _deg2rad(self, deg):
        # 角度转弧度
        return deg * math.pi / 180.0
    
    def set_origin(self, lat, lon):
        """
        设置GPS原点（经纬度/度）
        lat：纬度（度），北为正
        lon：经度（度），东为正
        """
        self.origin_lat = self._deg2rad(lat)
        self.origin_lon = self._deg2rad(lon)
        self.is_initialized = True
        print(f"[GPSConverter]原点已设置：lat={lat:.6f}, lon={lon:.6f}")

    def convert(self, lat, lon):
        """
        将GPS经纬度转换为world坐标系下的(x,y)

        lat：当前纬度（度）
        lon：当前经度（度）

        返回：
        (x,y):world坐标系下的位置（米）
        x = 东向位移
        y = 北向位移
        （从原点算起）
        """
        # 如果还未初始化，用第一帧数据作为原点
        if not self.is_initialized:
            self.set_origin(lat, lon)
            # 第一帧GPS读数对应车辆起点位置
            return self.vehicle_start_x, self.vehicle_start_y
        
        # 转为弧度
        lat_rad = self._deg2rad(lat)
        lon_rad = self._deg2rad(lon)

        # 计算经纬度差（弧度）
        d_lat = lat_rad - self.origin_lat
        d_lon = lon_rad - self.origin_lon

        # 将经纬度差转为米制偏移
        # 北向位移（米）：沿经线方向，1弧度纬度约等于R米
        dy = d_lat * self.EARTH_RADIUS

        # 东向位移（米）：沿纬度方向，考虑纬度缩放
        # 在原点纬度处，经线圆半径=R*cos（lat）
        dx = d_lon * self.EARTH_RADIUS * math.cos(self.origin_lat)

        # world坐标系：x=东，y=北
        world_x = self.vehicle_start_x + dx
        world_y = self.vehicle_start_y + dy

        return world_x, world_y
    
# # 简单测试（单独运行文件时执行）
# if __name__ == '__main__':
#     converter = GPSConverter()
#     # 假设第一帧是起始位置
#     x,y = converter.convert(31.2304, 121.4737)
#     print(f"第一帧（原点）：x={x:.2f},y={y:.2f}")
#     # 向北移动约10米
#     x,y = converter.convert(31.2304 + 10/111320, 121.4737)
#     print(f"向北10米：x={x:.2f},y={y:.2f}")