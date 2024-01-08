# FASTLIO_SAM_LC
1. 建图线程添加GTSAM做回环
2. 添加定位线程用于基于已知地图的重定位
3. 添加首帧重力对齐(用于传感器非水平放置情况)
4. 目前暂时支持MID_360的传感器(买不起其他传感器)

## 环境说明
```text
ubuntu20.04
ros noetic
pcl 1.10
```

## 编译依赖
1. livox_ros_driver2
2. gtsam (noetic版本 可以使用 sudo apt install ros-noetic-gtsam 直接安装，其他版本可能需要独立安装)
3. pcl

## 启动脚本
1. 建图线程
```shell
roslaunch fastlio mapping.launch
```
2. 定位线程
```shell
roslaunch fastlio localize.launch
```

## 服务脚本
1. 保存地图
```shell
rosservice call /save_map "save_path: 'you_pcd_save_path.pcd'
resolution: 0.0"
```
**目前resolution没用(需要降采样，可离线自行降采样)**

2. 重定位
```shell
rosservice call /slam_reloc "{pcd_path: 'you_pcd_path.pcd', x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}" 
```