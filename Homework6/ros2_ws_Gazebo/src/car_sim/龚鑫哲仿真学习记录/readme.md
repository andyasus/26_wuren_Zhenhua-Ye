这次任务并没有全部完成。

仿真的确困难重重，一开始并没有找到好的教程，官网上的也看不懂，于是直接让deepseek生成了一份urdf的demo

选择用手抄的方式去理解每一步的含义和作用。（因为手写更方便我补充某些地方的定义，不是为了显得自己用功。。。）

之后懂了大致含义和作用，开始学网上的教程。但未能彻底理解。

而在终于编写完urdf文件后，运行又很困难，成功导入gazebo之后，又发现传感器无法发送数据（这也是这次任务未完成的部分）

本次任务仅完成了对urdf文件的编写，world的编写。能成功把模型导入gazebo中并且各个部件连接正确

在最后，由于始终无法让传感器发送数据，所以直接让ai对所有文件进行了修改，但仍无法解决。（当前显示的是自己写的，除了阿克曼的那部分）

总之，对ai的使用量还是很大的。


![alt text](058ba004fa6e8d4314e33dfbe647e790.jpg)
![alt text](6ade5b90337b27566b104a6f444f3994.jpg)
![alt text](1e872416786b99805027faa7bc4b8e4b.jpg)

![alt text](image.png)

.

![alt text](image-1.png)
![alt text](image-2.png)

.


# 传感器：

一些传感器的定义库

https://gazebosim.org/libs/sensors/

  ### 对于每个传感器都要有以下代码：

  ![alt text](image-5.png)
  ### always_on 表示一直开启，若为false,则会在订阅后才开启
  ![alt text](image-3.png)

.

 ### 相机特有： 
   ![alt text](image-6.png)
   ![alt text](image-4.png)


 ### 雷达特有：
 ![alt text](image-7.png)

 ### <horizontal>是水平扫描

     <vertical>是垂直扫描

     <samples>是指雷达转一圈，一共发射并接受多少条激光束，数越大，分辨率越高，计算压力越大

     <resolution>是在这个水平层上，垂直方向有几个子采样层，一般为1

     <min_range>和<max_range>是扫描范围的最小和最大范围，单位为弧度，用-3.14159和3.14159，就能扫描360度

 ### <range>是扫描范围

     <min>是指最近检测距离，低于这个值，扫描不到

     <max>是指最远检测距离，高于这个值，扫描不到
     <resolution>是距离分辨率，定义扫描精度，单位为米，比如输入为0.01，则如果实际数据为2.022，则会输出2.02



# gps ,imu 磁力计（他们不需要特定子标签）（也可几何定义）

  ![alt text](image-9.png)

  ![alt text](image-10.png)


.

![alt text](image-11.png)

