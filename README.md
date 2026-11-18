# SCOUT Mini 室内自主导航

基于 ROS 2 Humble、Livox Mid-360 和 SCOUT Mini。项目使用 FAST-LIO2 建图、AMCL 定位，以及本项目实现的矩形 A* 全局规划和 DWA 局部规划。依赖、实车接线、节点说明和启动命令见 [项目完整说明书](SCOUT_Mini_自主导航项目完整说明书.md)。

`maps/fastlio_nav_02.yaml` 及其 PGM 是当前导航用的二维地图。原始三维 PCD、测试记录、编译产物和上游演示资料另存。需要新地图时，按说明书重新采集和转换。部署命令假设克隆到 `~/scout_ws`。

## 克隆后如何复刻

使用 Ubuntu 22.04 和 ROS 2 Humble。先按[完整说明书第 3 节](SCOUT_Mini_自主导航项目完整说明书.md)安装系统依赖和 Livox-SDK2，再从源码构建。仓库包含 14 个可由 colcon 发现的软件包，其中 `scout_ros2/` 内有 `scout_base`、`scout_description`、`scout_msgs` 三个包，`FAST_LIO_ROS2/` 对应的 ROS 包名是 `fast_lio`。ikd-Tree 源码已随 FAST-LIO2 附带。

```bash
# 把示例地址换成仓库的实际地址，新目录名与说明书统一为 scout_ws。
git clone https://github.com/OWNER/REPO.git ~/scout_ws
cd ~/scout_ws
source /opt/ros/humble/setup.bash
colcon list --base-paths src
colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
ros2 launch scout_bringup autonomy_system.launch.py --show-args
```

`build/`、`install/` 和 `log/` 由编译或运行生成。已附带的 YAML 和 PGM 可用于重现当前地图上的软件配置。换场地时重新建图，换设备时按实物配置 IP、CAN 接口、雷达外参和车体尺寸。完整实车复刻需要相应硬件和现场调试。

## 第三方源码

`src/` 保留了本项目运行所用的源码快照，包括对 FAST-LIO2、Livox 驱动和 Scout Mini 模型的本地修改。各上游项目保留各自的许可证和版权声明。

| 目录 | 上游和基准版本 |
| --- | --- |
| `FAST_LIO_ROS2` | [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)，`2fffc57` |
| `FAST_LIO_ROS2/include/ikd-Tree` | [hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree)，`e2e3f4e` |
| `livox_ros_driver2` | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)，`2144554` |
| `scout_ros2` | [agilexrobotics/scout_ros2](https://github.com/agilexrobotics/scout_ros2)，`bf0110f` |
| `ugv_sdk` | [agilexrobotics/ugv_sdk](https://github.com/agilexrobotics/ugv_sdk)，`f2704ea` |
