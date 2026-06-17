打开终端
cd gazebo+大作业/percep_node+track/test  (进入工作目录)

ros2 run robot_state_publisher robot_state_publisher vehicle_tf.urdf


打开新终端

rviz2

然后点击左下方的add，选择TF,然后点击ok。接着找到Global Options（在主页面上），把Fixed Frame改为world
这样就可以在rviz2中可视化车和各个传感器之间的相对位置了