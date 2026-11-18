# SCOUT Mini + Mid-360：已保存地图的定位配置与操作说明书

本次配置完成“在已有地图上定位”。启动和实车检查由你手动执行。
没有启动自动规划、局部避障或底盘速度控制；没有在你的电脑上运行本说明中的硬件启动命令。

## 1. 现在处于哪个阶段

你已经用 FAST-LIO2 保存三维地图，并导出了 `maps/fastlio_nav_02.yaml/.pgm`。
本次让机器人知道“我在这张地图的哪个位置，朝哪个方向”。

```text
之前：雷达 + IMU → FAST-LIO2 → 三维 PCD → 离线二维栅格图
本次：已保存二维图 + 当前二维扫描 + 轮式里程计 → AMCL → 地图中的车辆位置
以后：位置 + 目标 → 全局规划 → 局部避障/跟踪 → 底盘速度控制
```

本次运行 AMCL 定位时不同时启动 FAST-LIO2。
这是在 FAST-LIO2 建出的地图上使用另一种定位方法，不是给 FAST-LIO2 本身添加 PCD 重定位。
若要使用纯三维 PCD 重定位，需要另接点云匹配/重定位模块；本次没有实现该模块。

## 2. 地图与实车尺寸

默认地图：`/home/robot/scout_ws/maps/fastlio_nav_02.yaml`。

| 参数 | 数值 |
| --- | --- |
| 地图分辨率 | 0.05 m/格 |
| 已测整车最高点（最新） | 0.60 m |
| 地图障碍筛选高度 | 离地 0.10～0.75 m |
| 雷达原点离地高度 | 0.405 m |
| base_link 离地高度 | 0.079 m |
| 雷达相对 base_link | x=0.130、y=0、z=0.326 m |
| 雷达安装 pitch | +0.523599 rad（约30°，沿用安装假设） |

地图有大面积灰色未知区域。白色是有地面观测的区域，不等于已通过整车通行验证。
障碍尚未按车体外轮廓膨胀，低于 0.10 m 的障碍没有完整覆盖。
当前先在地图结构清楚、已知区域内验证定位，不把整张图认作已可自动通行。

## 3. 新手理解 AMCL

### 为什么重启后还需要定位

轮式里程计通常从本次启动的 `(0,0,0)` 开始。这个零点不一定是上次建图的零点。
因此即使已经有地图，也不能直接把当前位置当成地图原点。

### 粒子可以理解为许多“位置猜测”

AMCL 会维护许多带有位置与朝向的假设，叫粒子。
车移动时，轮式里程计让这些假设一起移动；考虑车轮打滑和测量误差，它们会产生分散。
收到扫描后，AMCL 对照地图：哪些假设下，实际扫到的墙与地图墙更吻合，就提高这些假设的权重。
反复进行后，位置估计通常会集中到更符合观测的位置。

因此需要：

1. 已有的地图 `/map`。
2. 当前的雷达扫描 `/scan`。
3. 连续的相对运动 `odom → base_footprint`（通过车体固定关系得到）。
4. 启动时大致的位置和朝向。本次通过 RViz 的 **2D Pose Estimate** 给出。

AMCL 不负责开车，也不会因为点击初始位置就让车辆移动。

## 4. 本次 TF 与坐标关系

```text
map                         导出的二维地图坐标，由 AMCL 确定车辆在其中的位置
 └─ odom                    本次启动的轮式里程计坐标
     └─ base_link           车辆轮轴中心，z=0.079 m
         ├─ base_footprint  车体在地面的参考点，z 比 base_link 低0.079 m
         └─ livox_level    位于雷达原点、与车体轴向一致
             └─ livox_frame
```

- `map → odom`：由 AMCL 发布，用地图修正累计漂移，初始定位前可能不存在。
- `odom → base_link`：新只读 CAN 里程计节点发布。
- `base_link → base_footprint/livox_level/livox_frame`：已有机器人模型发布。

AMCL 的 `base_frame_id` 特意使用 **base_footprint**：它是地面上的二维参考点。
这避免把离地 79 mm 的 base_link 错当作 z=0 的地面点。
车辆模型仍然沿用 base_link 为父、base_footprint 为子，不需要改原来的 Xacro 根结构。

导出地图经过地面水平校正，与原 FAST-LIO2 的在线 map 坐标存在一个固定变换，记录在 `.frames.yaml`。
本次 AMCL 直接把导出图片当作新的 map，通过人工初始位姿和扫描匹配重新建立 map→odom，
所以无需也不能再额外发布旧 FAST-LIO2 的 map→camera_init 关系。
机器人到雷达的静态安装关系仍需要真实标定；离线地面校正不能替代雷达安装标定。
当前实时投影假设车在平地上，没有用 IMU 在线补偿车体俯仰/侧倾。

## 5. 改了哪些文件

| 文件 | 用途 |
| --- | --- |
| `src/scout_bringup/launch/localization_system.launch.py` | 新的一键定位入口 |
| `src/scout_localization/launch/fastlio_grid_localization.launch.py` | 地图服务、AMCL 和生命周期管理 |
| `src/scout_localization/scout_localization/can_odometry.py` | 只接收 AGX_V2 CAN 反馈并计算里程计 |
| `src/scout_localization/config/amcl_fastlio_grid.yaml` | 新地图的 AMCL 初始参数 |
| `src/scout_localization/config/scan_localization.yaml` | 实时点云转二维扫描参数 |
| `src/scout_localization/config/localization.rviz` | 地图、红色扫描和定位显示配置 |
| `src/scout_localization/setup.py` | 注册 `can_odometry` 可执行入口 |
| `src/scout_localization/package.xml`、`src/scout_bringup/package.xml` | 补齐运行依赖 |

旧 FAST-LIO2 建图入口、旧二维建图和旧 navigation_system.launch.py 均保留。
**本阶段使用 localization_system.launch.py，不启动旧 navigation_system.launch.py。**

## 6. 为什么新增只读 CAN 节点

原 Scout 驱动启动时调用 `EnableCommandedMode()`，并订阅 `/cmd_vel`。
本阶段只需遥控器开车、电脑读取反馈，因此使用独立的接收节点：

- 没有 `socket.send/sendto/sendall` 调用。
- 没有 `/cmd_vel` 或其他控制指令订阅。
- 不请求切换底盘控制模式。
- 不发送灯光或运动控制帧。
- 仅按本地 ugv_sdk 的 AGX_V2 定义读取 0x211、0x221。

### `decode_frame()`

系统帧 `0x211` 提供底盘状态、控制模式、电压和故障码。
运动帧 `0x221` 提供带符号的线速度、角速度等反馈。
代码中的：

```python
linear, angular, lateral, steering = struct.unpack('>hhhh', payload)
```

`>` 表示大端字节序；`h` 表示有符号16位整数。除以1000后得到 m/s 和 rad/s。
普通四轮差速模式使用纵向线速度和偏航角速度。
本节点针对此前已确认的 **AGX_V2**，不支持把 AGX_V1 帧直接当作 V2 解码。

### `integrate_pose()`

直行时，速度乘时间得到距离，再根据车头方向分解为 x、y 位移。
转弯时使用恒定线速度/角速度的圆弧积分，而不是简单把转弯当直行。
它累计的是反馈速度，因此遥控移动时也能更新位置。
但车轮打滑仍会产生误差，AMCL 的任务就是结合地图观测修正这类累计漂移。

### 数据异常处理

- 初次收到数据时，不把启动前时间积分成位移。
- 车辆异常或故障码非零时暂停里程计输出。
- 反馈断流后不无限重复发布看似新鲜的旧 TF。
- 超过0.15秒的积分间隔不补算未知运动，报告 `integration_gaps`。
- 系统状态过期或运动反馈过期，在诊断话题报告异常。
- 反馈恢复后重新接续，断流期间的真实位移无法恢复，需要重新检查定位或设置初始位姿。

本阶段没有发布电机电流/温度等完整 scout_status，使用单独诊断话题：
`/localization/can_diagnostics`。

## 7. 点云投影为什么是这两个高度

定位时雷达驱动使用 PointCloud2，方便 `pointcloud_to_laserscan` 直接处理。
原始三维点先变换到 `livox_level`，再选取指定高度的点。

```text
地面障碍下限 0.10 − 雷达离地 0.405 = −0.305 m
地面障碍上限 0.75 − 雷达离地 0.405 = +0.345 m
```

所以 scan_localization.yaml 中：

```yaml
target_frame: livox_level
min_height: -0.305
max_height: 0.345
```

这与离线地图的10～75 cm候选障碍范围对应。
每个水平角度选取范围内的最近点，得到二维 LaserScan，供 AMCL 与地图比较。
角度间隔为0.5°，约720个方向；range_min=0.35 m，range_max=12 m。
这些都是初始配置。地面倾斜、玻璃、车体遮挡、低障碍仍会影响结果。

定位入口的雷达输出是 PointCloud2；FAST-LIO2 建图入口的输出是 CustomMsg。
同一时间只能启动其中一种雷达驱动，不能两个入口同时运行。

## 8. 操作命令：由你手动执行

### 先结束其他 ROS 系统

关闭 FAST-LIO2 建图、旧二维建图、原 scout_base、单独雷达、旧 lidar_tf 等启动终端。
只保留终端本身即可。确保没有第二个节点在发布 odom→base_link 或 map→odom。

### 雷达网络

使用当前已验证的 USB 网卡时：

```bash
ip -br addr
ip route get 192.168.1.178
ping -I enx00e04c10016b -c 4 192.168.1.178
```

若地址/专用路由丢失，再执行：

```bash
sudo ip link set enx00e04c10016b up
sudo ip addr replace 192.168.1.5/32 dev enx00e04c10016b
sudo ip route replace 192.168.1.178/32 dev enx00e04c10016b src 192.168.1.5
```

若换了网口，先确认接口名；不要把同一个192.168.1.5配置在多个网卡。

### 底盘 CAN

底盘正常通电，遥控器保持手动档，摇杆回中：

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

CAN 的 `ERROR-ACTIVE` 是 CAN 控制器的正常错误计数状态名，不等于底盘故障码。
若没有 `can0`，先检查 USB-CAN 设备连接。

### 一键启动定位

```bash
source /opt/ros/humble/setup.bash
source ~/scout_ws/install/setup.bash

ros2 launch scout_bringup localization_system.launch.py \
  map:=/home/robot/scout_ws/maps/fastlio_nav_02.yaml
```

保持该终端运行。它启动：

1. Mid-360 PointCloud2 驱动。
2. 机器人固定 TF。
3. CAN 只读里程计。
4. 点云转 /scan。
5. 地图服务与 AMCL。
6. 定位 RViz。

**不启动 FAST-LIO2、不订阅速度命令、不启动规划或车辆控制节点。**

### 设置初始位置

新 RViz 已设 Fixed Frame=map，并显示保存地图和红色实时扫描。

1. 车先停在平地、地图白色已知区域中。
2. 参照周围墙面/物体，判断车辆在地图中的大致位置。
3. 点击工具栏 **2D Pose Estimate**。
4. 在地图上的车辆位置按下鼠标，向车头朝向拖出箭头，松开。
5. 等待几秒，看红色扫描是否大致贴住黑色地图边界。

不要点击2D Goal Pose来测试定位，它属于后续导航用途；本配置没有目标控制。
没有初始化前，map到车体的TF可能尚不存在，扫描暂时无法在map坐标显示，这是需要先给初始位姿的状态。

### 另开终端检查

每个新终端先 source ROS 与工作区环境。

```bash
ros2 topic echo /localization/can_diagnostics --once
ros2 topic echo /odom --once
ros2 topic hz /scan
```

查看频率后按 Ctrl+C，再执行：

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
```

诊断 KeyValue 中重点看：

```text
vehicle_state = 0
control_mode = 3
error_code = 0x0
integration_gaps = 0 或不持续增长
```

`/amcl_pose` 通常随扫描和运动更新，不是固定50Hz。
如果长时间没有输出，先检查初始位姿、/scan和odom TF。

### 遥控验证

红色扫描与地图结构大致重合后，再低速前进一小段、缓慢转弯。
观察车辆位置是否跟着变化，以及固定墙面的扫描是否仍与地图对应。
最后回到起点附近，检查位置和朝向的一致性。
本阶段只需遥控移动，不启用任何自动速度。

退出时在定位启动终端按 Ctrl+C。
已有地图不会因为定位过程更新，无需再调用 /map_save。

## 9. 常见现象怎么查

| 现象 | 优先检查 |
| --- | --- |
| 报 CAN bind / No such device | can0 是否存在、是否已按500k开启 |
| 没有 /odom | /localization/can_diagnostics；CAN反馈、车辆故障、协议版本 |
| /odom有输出但车不动数值也不变 | 静止时正常，缓慢遥控移动后再看 |
| 底盘不能遥控 | 本节点不切控制模式；看遥控器故障码与手动档。不要混用原底盘驱动 |
| 没有 /scan | 点云格式应为PointCloud2，雷达数据、静态TF与投影高度 |
| 一开始地图有、扫描没有 | 先设置2D Pose Estimate，再查map→odom |
| 扫描整体错开 | 初始位置/朝向、静态雷达TF、地图是否对应现场 |
| 一走就偏 | 轮滑、地面不平、CAN断流、地图与扫描匹配差、标定误差 |
| 灰色区域很多 | PCD缺少地面观测；不是AMCL能自动填补的区域 |
| 缺地图文件 | 检查传入yaml及其中image指向的pgm |

## 10. 参数与代码解释

### `localization_system.launch.py`

`defaults` 提供地图路径、网卡/CAN名称相关选项和安装位姿默认值。
`LaunchConfiguration` 在你运行ros2 launch时读取参数，`ParameterValue(..., value_type=bool)`确保true/false按布尔值传给节点。
`IfCondition` 允许回放时关闭真实雷达或真实CAN节点。

例如仅使用录制的雷达与里程计数据时：

```bash
ros2 launch scout_bringup localization_system.launch.py \
  start_lidar:=false start_odometry:=false use_sim_time:=true
```

这只是启动选项，仍需要另外播放包含PointCloud2、odom以及对应TF的录制文件和/clock。
本项目之前没有保证录制了这些数据，因此不是现成可回放的数据包。
只读CAN接收器用真实单调时钟积分，不支持use_sim_time=true；回放必须关闭它。

### `fastlio_grid_localization.launch.py`

`check_map()` 在运行前确认地图YAML、图像存在，分辨率和占用阈值合理。
`map_server` 加载地图，不根据实时扫描改地图。
`amcl` 接收地图、扫描、里程计TF，估计map中的车辆位姿。
`lifecycle_manager` 将地图服务和AMCL从未配置/未激活状态转为工作状态。
它不做车辆路径规划。

### `amcl_fastlio_grid.yaml`

- base_frame_id=base_footprint：地面二维车体参考点。
- min_particles=500、max_particles=2000：位置假设数量的范围。
- max_beams=100：每次用于匹配的扫描方向数量。
- update_min_d=0.05：移动约5cm后允许更新。
- update_min_a=0.05：旋转约0.05rad后允许更新。
- alpha1～alpha4：轮式运动误差模型初值，未通过实车标定。
- laser_model_type=likelihood_field：根据扫描点与地图障碍的距离评价位置假设。
- sigma_hit=0.2：匹配噪声模型初值，不表示本车已达到20cm精度。
- set_initial_pose=false：不写死一个起点，由你通过RViz给出。
- save_pose_rate=0.5：采用正常的位姿参数保存频率；本机Humble版本内部计算1/rate，设为0会导致配置阶段时间溢出。
- tf_broadcast=true：由AMCL发布map→odom。

### 地图白色不等于可直接自动行驶

这套配置只完成定位链路，后面仍需处理：
- 车体外轮廓与障碍膨胀。
- 实时障碍地图和局部避障。
- 全局规划、路径跟踪和速度输出。
- 低矮障碍、悬空障碍与传感器盲区。

## 11. 验证范围

已做的检查记录在本文末尾；实际未启动CAN接收器、雷达、AMCL或底盘。
离线检查验证软件解析、积分、配置与安装，不代替实际车辆上的定位精度验证。

本次已完成的具体检查：

- `scout_localization` 与 `scout_bringup` 构建安装成功。
- 新入口 `ros2 launch ... --show-args` 成功加载。
- 地图YAML和PGM路径预检查成功；缺失地图被明确拒绝。
- 所需可执行文件与RViz插件在本机存在；YAML/XML/Python语法检查通过。
- 用内存中的模拟CAN帧验证：大端字节序、负速度、缩放单位、状态和故障码。
- 验证直线与圆弧积分，以及首次接收、积分间隔过大、故障、过期与恢复处理。
- 用消息对象验证odom和TF的时间戳、坐标名、速度字段及0.079m轮轴高度。
- 实时扫描高度加回0.405m后，与离线地图0.10～0.75m范围一致。
- 结构检查确认新CAN节点没有发送方法调用或控制指令订阅。

这些检查没有创建真实CAN套接字，也没有运行AMCL/RViz/真实雷达进程。
尚需你按第8节手动验证：CAN实际反馈、AMCL激活、初始位姿、扫描与地图重合及遥控行驶时定位稳定性。

## 2026-11-17 实机启动问题修正

实机日志发现 AMCL 配置阶段报 `Input t_sec is too large or too small for tf2::Duration`。
已将 `amcl_fastlio_grid.yaml` 的 `save_pose_rate` 从0修正为0.5 Hz。
原配置导致AMCL未配置、地图服务停留inactive；需要关闭定位launch后重新启动。
同时本次重启后的can0为DOWN，需先手动按500000 bitrate开启。
本次只读取运行状态和修正文件，没有替用户更改CAN状态、生命周期或重启进程。

## 2026-11-17 CAN 接收过滤器修正

现象：candump有反馈，但只读里程计诊断中的vehicle_state等始终unknown，/odom没有输出；AMCL丢弃扫描，map TF无法建立。
根因：此前CAN_RAW_FILTER掩码误用了0xE00007FF，包含CAN_ERR_FLAG。Linux会将带此标志的掩码送入错误帧接收路径，不能用它筛选普通数据帧。
修正：使用CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK，即0xC00007FF。解码函数仍拒绝扩展帧、远程帧和错误帧。
只读实测对比：同一can0、同一1.5秒窗口，旧掩码收到0帧，新掩码收到83帧。未发送CAN控制。
安装的Python模块已能导入修正值，无需重新编译；正在运行的定位进程须由你Ctrl+C后重启。
重启后先检查/localization/can_diagnostics和/odom，再用2D Pose Estimate初始化。
如果odom正常后仍持续出现时间缓存错误，才进一步测量雷达和TF时间戳，不能凭之前的错误直接修改时钟或扫描时间戳。

## 最新尺寸与局部预览阶段

用户更新整车尺寸为长0.61 m、宽0.56 m、高0.60 m，替代早前0.65 m高度记录。
现有地图与AMCL扫描仍保留0.10～0.75 m高度范围，以保持已经验证的定位配置。
局部预览另外使用0.05～0.75 m的/local_scan，并不改变AMCL的/scan。
详见《局部避障与路径跟踪_预览配置说明书.md》。
