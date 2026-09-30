# hikrobot_camera

整理后的 Hikrobot MVS ROS 2 相机驱动。

这个版本保留原始方案：

```text
Hikrobot MVS
    |
    | MV_CC_GetImageBuffer
    v
MVS native frame
    |
    | MV_CC_ConvertPixelType
    v
BGR8
    |
    | cv::Mat + cv_bridge
    v
sensor_msgs/msg/Image
    |
    v
/image_raw
```


## 目录结构

```text
hikrobot_camera/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/hikrobot_camera/
│   └── camera_node.hpp
├── src/
│   ├── main.cpp
│   └── camera_node.cpp
├── launch/
│   └── camera.launch.py
└── config/
    └── camera.yaml
```

## 环境

适配当前使用的 Ubuntu 24.04 + ROS 2 Jazzy。

MVS SDK 默认路径：

```text
/opt/MVS/include
/opt/MVS/lib/64
/opt/MVS/bin
```

需要 ROS/OpenCV 依赖：

```bash
sudo apt update
sudo apt install \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  libopencv-dev
```

## 编译

```bash
cd ~/桌面/assignment3-ROS2

rm -rf build install log

source /opt/ros/jazzy/setup.zsh

export LD_LIBRARY_PATH=/opt/MVS/bin:/opt/MVS/lib/64:$LD_LIBRARY_PATH

colcon build \
  --symlink-install \
  --packages-select hikrobot_camera
```

## 运行

```bash
cd ~/桌面/assignment3-ROS2

source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

export LD_LIBRARY_PATH=/opt/MVS/bin:/opt/MVS/lib/64:$LD_LIBRARY_PATH

ros2 launch hikrobot_camera camera.launch.py
```

## 话题

```bash
ros2 topic list
ros2 topic info /image_raw -v
```

默认发布：

```text
/image_raw
encoding: bgr8
QoS: Best Effort
depth: 1
```

## 动态参数

例如：

```bash
ros2 param set /hikrobot_camera exposure_time 7000.0
ros2 param set /hikrobot_camera gain 5.0
ros2 param set /hikrobot_camera frame_rate 60.0
```

## RViz2

```bash
rviz2
```

添加 `Image`：

```text
Topic: /image_raw
Reliability: Best Effort
```



