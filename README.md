# 用于ros调试时候的可视化工具

# 功能

## 包含功能

* 将lidar的pointcloud结合tf信息，计算出其在任何相机像素坐标系内的投影点，并通过ImageMarker消息发布出来

* 将以autoware DetectedObjectArray格式发布出来的3D检测结果后处理后通过MarkerArray发布出来显示（3D视图中显示出来）

## 待添加功能

* 将以autoware DetectedObjectArray格式发布出来的3D检测结果后处理后通过ImageMarkerArray发布出来显示（2D视图中显示出来）


## 环境准备
```bash
sudo apt install ros-noetic-foxglove-msgs

python3 -m pip install --user git+https://github.com/windzu/pypcd.git

pip install scipy


```

# 用法

* 通过test.py将config文件夹下的camera_info.yaml 和 static_tf_info两个文件内容都发布出去

* main.py是整


