# cmu_local_planner

功能包简介：

1. 添加了详细注释的CMU局部规划导航代码
2. 通过订阅地形分析后的点云数据，定位信息，导航点信息，规划局部路径并避障

## 使用方法

### 下载并编译功能包

```bash
cd catkin_ws/src
git clone https://github.com/LiiXZ/cmu_local_planner.git
cd ..
catkin_make
```

### 启动节点

```bash
roslaunch local_planner my_local_planner.launch
```

### 使用时需要修改的参数

修改 `launch/local_planner.launch` 中的：

```xml
  <arg name="points_topic" default="/rslidar_points" />
  <arg name="state_topic" default="/us_liorf_localization/mapping/base_odometry" />
  <arg name="odom_topic" default="/us_liorf_localization/mapping/base_odometry" />
```
