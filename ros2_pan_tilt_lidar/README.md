# ROS2 Pan Tilt Lidar

## 项目简介
该项目实现了一个基于ROS2的激光测距和云台控制系统。通过控制云台的运动，系统能够进行3D扫描，并实时获取测距数据。

## 文件结构
```
ros2_pan_tilt_lidar
├── pan_tilt_lidar
│   ├── __init__.py          # 包初始化文件
│   ├── node_main.py         # ROS2节点的入口点
│   └── utils.py             # 辅助函数
├── launch
│   └── pan_tilt_lidar.launch.py  # ROS2启动文件
├── package.xml              # 包配置文件
├── setup.py                 # 包安装和依赖项设置
├── setup.cfg                # 包安装选项和元数据配置
├── resource
│   └── ros2_pan_tilt_lidar  # 资源文件目录
└── README.md                # 项目文档和使用说明
```

## 使用说明
1. **安装依赖**: 确保已安装ROS2及相关依赖项。
2. **构建包**: 在工作空间中运行以下命令以构建包：
   ```
   colcon build
   ```
3. **运行节点**: 使用以下命令启动节点：
   ```
   ros2 launch pan_tilt_lidar pan_tilt_lidar.launch.py
   ```
4. **功能**: 节点会订阅来自其他节点的信号，并在接收到信号时自动开始激光测距和云台控制的执行。

## 贡献
欢迎任何形式的贡献！请提交问题或拉取请求。

## 许可证
该项目遵循MIT许可证。