# 环境定义：
  
   坐标轴定义： 全局坐标（world）为东北天

               其中，x轴为东，y轴为北，z轴为天

              对于车辆（base_link）,为前左上

               其中，x轴为前，y轴为左，z轴为上
    
    车辆起始位置：（x=0,y=-15,z=0）,且朝北（Yaw = 1.5708）


所有节点按照下方的规定来命名，不可擅自更改，如有更改，要在群里通知大家

# 接口定义：

 ### 相机： 
       话题名：/camera/image_raw

       消息类型：sensor_msgs/msg/Image

 ### 雷达：
      
       话题名：/scan
       
       消息类型：sensor_msgs/msg/LaserScan

 ### gps：（获取经纬度）
       
       话题名：/gps/fix
       
       消息类型：sensor_msgs/msg/NavSatFix

 ### IMU：
       
       话题名：/imu/data
       
       消息类型：sensor_msgs/msg/Imu

 ### 磁力计：
  
       话题名：/mag/data
       
       消息类型：sensor_msgs/msg/MagneticField

 ### 里程计：
  
       话题名：/odometry
       
       消息类型：nav_msgs/msg/Odometry

 ### 控制指令：（速度+前轮转向角）

       信息传向：ros2到gazebo  (除此之外，其他都是gazebo到ros2)
    
       话题名：	/cmd_ackermann
       
       消息类型：ackermann_msgs/msg/AckermannDrive


# 各坐标系定义：
 
 ### 坐标系名称（frame_id）:
     
     world                          全局坐标系             无父坐标系
     base_link                      车辆坐标系             父坐标系：world
     camera_link                    相机坐标系             父坐标系：base_link
     lidar_link                     雷达坐标系             父坐标系：base_link
     gps_link                       gps坐标系             父坐标系：base_link
     imu_link                       imuU坐标系            父坐标系：base_link
     magnetometer_link              磁力计坐标系           父坐标系：base_link
     odometry_link                  里程计坐标系           父坐标系：base_link


 ### 传感器安装：
     
     camera_link          位置（相对于base_link平移）：（x=0.2,y=0,z=0.1）
     lidar_link           位置（相对于base_link平移）：（x=0.3,y=0,z=0.15）
     gps_link             位置（相对于base_link平移）：（x=0,y=0,z=0.05）
     imu_link             位置（相对于base_link平移）：（x=0.05,y=0,z=0.02）
     magnetometer_link    位置（相对于base_link平移）：（x=-0.05,y=0,z=0.12）

# 车的参数：
   
    轴距：0.3m

    最大线速度：5m/s

    车宽：0.2m

    总车长:0.43