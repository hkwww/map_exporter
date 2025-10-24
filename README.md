# Map Exporter

## 1. 简介

`map_exporter`仓库用于将`LAS`点云地图转为`mfla`系统和调度系统可用的地图格式。

## 2. 安装及使用

### 2.1 依赖安装

#### ROS Noetic

#### PDL
```bash
  sudo apt-get install libpcl-dev pdal libpdal-dev
```

### 2.2 使用方法

```bash
  cd catkin_ws
  source devel/setup.bash
  roslaunch mfla_monitor monitor.launch
```