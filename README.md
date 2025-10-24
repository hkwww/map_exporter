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

### 2.2 编译方法

```bash
  mkdir -p catkin_ws/src
  cd catkin_ws/src
  git clone https://github.com/hkwww/map_exporter.git
  cd ..
  catkin_make
```

### 2.3 使用方法

**参数修改**
修改[export_map.launch](./launch/export_map.launch)中的参数

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="map_exporter" type="map_exporter_node" name="map_exporter_node" output="screen" respawn="false">
        <param name="las_file_path" value="/home/hkw/20251011102314752.las"/>
        <param name="output_path" value="/home/hkw/dataset/test_map"/>
    </node>
</launch>
```

- `las_file_path`：输入的`LAS`点云地图路径
- `output_path`：输出的地图文件夹路径

**运行程序**
```bash
  cd catkin_ws
  source devel/setup.bash
  roslaunch map_exporter export_map.launch
```

终端显示
```bash
  LAS File Path: /home/hkw/20251011102314752.las
  Output Path: /home/hkw/dataset/test_map
  Loaded 13753907 points from /home/hkw/20251011102314752.las
  Map exported successfully to: /home/hkw/dataset/test_map
```
即为导出成功，会在`output_path`路径下生成`mfla`系统和调度系统可用的地图文件。
```bash
  ~/dataset/test_map$ tree
  .
  ├── grid_map
  │   ├── map_image.pgm
  │   ├── map_image.png
  │   └── map.yaml
  ├── map.pcd
  └── map.yaml
```
