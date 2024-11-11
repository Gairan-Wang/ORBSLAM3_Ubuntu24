# Readme

orbslam3 基本结构

## 目录结构

```
.
├── CMakeLists.txt
├── inc
│   ├── Camera
│      ├── GemometricCamera.h
│      ├── PinholeCamera.h
│      ├── KannalaBrandt8.h
│   ├── Converter.h
│   ├── GemometircTools.h
│   ├── ImuTypes.h
│   ├── SerializationUtils.h
│   ├── Config.h
│   ├── Sim3Solver.h
│   ├── ORBextractor.h
│   ├── ORBmatcher.h
│   ├── ORBVocabulary.h
│   ├── Settings.h
│   ├── G2oTypes.h
│   ├── Frame.h
│   ├── KeyFrame.h
│   ├── Map.h
│   ├── MapPoint.h
│   ├── MapDrawer.h
│   ├── MLPnPsolver.h
│   ├── Tracking.h
│   ├── FrameDrawer.h
│   ├── Viewer.h
```

## 主要文件

- Converter.h: 转换器类，用于将 ORB-SLAM3 的输出转换为其他格式
- GemometircTools.h: 地球坐标系转换工具类
- SerializationUtils.h: 序列化工具类
- ImuTypes.h: 定义 IMU 相关数据结构和功能函数，IMU 数据的存储、管理和处理
- Config.h: 定义初始化配置类，用于读取配置文件并初始化 ORB-SLAM3
- Sim3Solver.h: 三角化求解器类，用于求解相机位姿
- ORBextractor.h: ORB 特征提取器类，用于提取图像中的特征点
- ORBmatcher.h: ORB 特征匹配器类，用于匹配特征点
- ORBVocabulary.h: ORB 词汇库类，用于管理 ORB 特征点的描述子
- Settings.h: 定义设置类，用于设置 ORB-SLAM3 的参数
- G2oTypes.h: 定义 G2O 优化器相关数据结构和功能函数，G2O 优化器的初始化、数据结构的管理和处理
- Frame.h: 定义帧类，用于管理图像帧的相关数据
- KeyFrame.h: 定义关键帧类，用于管理关键帧的相关数据
- Map.h: 定义地图类，用于管理地图的相关数据
- MapPoint.h: 定义地图点类，用于管理地图点的相关数据
- MapDrawer.h: 地图绘制类，用于绘制地图的相关数据
- MLPnPsolver.h: 多视图 PnP 优化器类，用于求解多视图 PnP 问题
- Tracking.h: 跟踪模块类，用于处理图像序列中的特征点，并进行特征点跟踪
- FrameDrawer.h: 帧绘制类，用于绘制帧的相关数据
- Viewer.h: 视图类，用于显示地图、帧、特征点、IMU 数据等信息

## 主要线程模块

- Tracking: 跟踪模块，用于处理图像序列中的特征点，并进行特征点跟踪
- LocalMapping: 局部地图构建模块，用于构建局部地图，包括局部地图点的插入、局部地图点的匹配、局部地图点的优化、局部地图的保存和更新
- Loop and map merging: 当加入一个新的关键帧时，在激活地图和 Atlas 地图中检测公共区域。如果属于公共区域则进行回环校正，否则作为新地图融合。在回环校正后，一个独立的线程会进行全局 BA，进一步优化地图。
- Atlas: 地图集成模块，用于将多个局部地图集成为全局地图，包括地图的融合、地图的保存和更新
