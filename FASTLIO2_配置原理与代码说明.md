# SCOUT Mini + Mid-360：FAST-LIO2 配置、原理和代码说明

这份文档对应本工作区本次修改。单位：距离用米，角度用弧度。

## 1. 先理解：现在换了什么

原来：

```text
Mid-360 → /livox/lidar（PointCloud2）→ 转成二维 /scan
底盘 CAN → /odom（轮式里程计）                ↓
                                SLAM Toolbox → /map（黑白二维地图）
```

现在：

```text
Mid-360 → /livox/lidar（CustomMsg，含逐点时间）─┐
        → /livox/imu（角速度、加速度）────────┤
                                         FAST-LIO2
                                             ├─ /Odometry：估计的 IMU 位姿
                                             ├─ /cloud_registered：配准后的当前三维扫描
                                             ├─ /Laser_map：累积三维点云地图
                                             └─ /path：运动轨迹
```

`/Odometry` 大写 O，与旧底盘的 `/odom` 不是同一个话题。
这套入口不启动 SLAM Toolbox，也不生成 `nav_msgs/OccupancyGrid` 类型的二维 `/map`。
`map` 坐标系和 `/map` 话题是两件事：有 map 坐标系，并不代表已有二维地图话题。

算法采用 HKU-MARS FAST-LIO2 的社区 ROS 2 移植版：
https://github.com/Ericsii/FAST_LIO_ROS2/tree/ros2
源码版本：`2fffc570a25d0df172720bac034fbdb6a13d2162`。
ikd-Tree 子模块版本：`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`。
它不是 HKU 官方提供的完整 ROS 2 导航系统，也没有在本次接入 EGO-Planner。

## 2. 新手理解 FAST-LIO2 的工作原理

### 点云：雷达看到的距离点

雷达对周围物体测距，把每个测量写成 x、y、z。例如墙上某一点在雷达前方两米、左边一米、上方半米，就可写成 `(2, 1, 0.5)`。
一帧里有许多这样的点，合起来叫点云。点云是物体表面被扫到的位置，并不是实体墙的完整模型。

单帧原始点云可能稀疏、有扫描纹路，也可能受到玻璃反射和遮挡影响。
截图中原来显示的是 `/livox/lidar`，仅看这一帧，无法证明定位算法发散。

### IMU：短时间里估计“怎么转、怎么动”

IMU 的陀螺仪测角速度，加速度计测比力（静止时也包含重力对应的读数）。
它更新快，但有噪声和偏置。只靠积分，位置误差会越来越大，因此不能独立代替稳定的定位。

### 预测：先用 IMU 推算当前状态

FAST-LIO2 先使用连续 IMU 数据预测位置、速度、姿态，同时维护陀螺仪偏置、加速度偏置和重力估计。
可以理解为先给出一个“现在大概到了哪里”的答案。

### 去畸变：同一帧的点并不是同时测到的

10 Hz 点云的一帧约跨越 0.1 秒。若车辆在这段时间内移动或转弯，帧头和帧尾的点是在不同位置测到的。
把它们直接当作同时测到，会让墙弯曲、变厚。
FAST-LIO2 利用逐点时间与 IMU 预测运动，把一帧中的点补偿到统一时刻。
这就是为什么本次把雷达输出改成 `CustomMsg`。
停车时的反射杂点、遮挡或原始扫描纹路，不能指望单靠去畸变消除。

### 匹配：把当前点云放到已有地图里

算法用预测姿态把新点变换到地图坐标，寻找附近的地图点，构造局部平面，计算新点到平面的距离。
例如新扫描的墙与旧地图墙没有对齐，就有误差；算法通过迭代误差状态卡尔曼滤波更新位姿，使二者更一致。
它同时考虑激光约束、IMU 预测与各自不确定性，而不是简单把两种位置取平均。

### ikd-Tree：快速查找地图附近的点

地图点数很多，不能每次从头遍历。ikd-Tree 是一种可增量更新的空间索引，帮助算法快速查找邻近地图点。
它不是另一套 SLAM 算法。

### 拼地图：在估计的位置上加入新扫描

位置估计后，再把新扫描加入地图。车辆走到新的区域，就得到更多表面点。
地图清楚的前提是输入、同步、标定和环境条件正常；更换算法并不保证所有场景都不漂移。
本次接入的 FAST-LIO2 是激光惯性里程计和增量建图，没有加入回环优化与已有地图重定位模块。
走一圈回到起点后不一定自动消除全部累计误差。

## 3. 这次改动的文件与用途

| 文件 | 改动及作用 |
| --- | --- |
| `src/FAST_LIO_ROS2/` | 新增 FAST-LIO2 ROS 2 源码和 ikd-Tree 子模块 |
| `src/scout_bringup/launch/mapping_system.launch.py` | 默认建图入口，改为启动 CustomMsg 雷达、机器人模型、FAST-LIO2 和新 RViz |
| `src/scout_mapping/launch/slam.launch.py` | 原来启动 SLAM Toolbox，现在加载 FAST-LIO2 参数、计算安装 TF、启动算法和 RViz |
| `src/scout_mapping/config/fastlio_mid360.yaml` | 为 Mid-360 配置输入话题、逐点时间、滤波、内置 IMU 外参和输出 |
| `src/scout_mapping/config/fastlio.rviz` | 新三维显示配置，观察配准扫描、地图、轨迹 |
| `src/scout_mapping/package.xml` | 声明 fast_lio、TF、RViz 与矩阵运算等运行依赖 |
| `src/scout_bringup/launch/mapping_2d.launch.py` | 保留原二维一键建图入口 |
| `src/scout_mapping/launch/slam_2d.launch.py` | 保留原 SLAM Toolbox 启动方式 |

旧二维地图 `maps/my_map*.pgm/yaml` 保留，导航入口尚未改接 FAST-LIO2。
机器人 Xacro 和静态安装 TF 文件沿用前面已测量的配置。
没有改雷达 IP、没有向底盘发送运动指令，也没有设置系统开机自启。

## 4. 一键启动文件逐段解释

打开：`src/scout_bringup/launch/mapping_system.launch.py`。

### `defaults`：把实测尺寸设为默认值

```python
defaults = {'x': '0.130', 'y': '0.0', 'z': '0.326',
            'lidar_pitch': '0.523599', 'lidar_yaw': '0.0',
            'use_sim_time': 'false', 'start_lidar': 'true'}
```

- x：雷达点云原点位于 base_link 前方 130 mm。
- y：左右居中。
- z：雷达高 405 mm，轮轴高 79 mm，因此相对高度为 326 mm。
- pitch：保留前倾 30°的配置。正负与实际雷达轴向有关，仍需检查真实地面是否水平。
- yaw：保留相对车头的偏航角 0；不代表已通过精密标定。
- use_sim_time：实机用 false；播放录制数据并使用 /clock 时可设 true。
- start_lidar：实机用 true；回放已经录制的输入时设 false，避免启动第二份雷达驱动。

这些是 launch 参数，不是每次都要手工填的值。

### `load()`：把一个启动文件包含到另一个启动文件里

```python
path = os.path.join(pkg(package), directory, filename)
```

`pkg(package)` 查 ROS 安装目录，不硬编码源码绝对路径。
`directory` 通常是 launch，而 Livox 驱动在这台机器上是 `launch_ROS2`。
`os.path.isfile()` 先检查文件存在，尽早报错，避免部分系统已经启动才发现路径错误。

### `msg_MID360_launch.py`：切换为逐点时间消息

```python
load('livox_ros_driver2', 'msg_MID360_launch.py', directory='launch_ROS2', ...)
```

该驱动启动文件已有 `xfer_format=1`，所以 `/livox/lidar` 改成 `livox_ros_driver2/msg/CustomMsg`。
与旧的 `rviz_MID360_launch.py`（xfer_format=0）相比，话题名字相同，消息类型改变。
CustomMsg 中有每点 xyz、反射强度、line、tag、offset_time；算法用 offset_time 知道每个点何时测到。
因此新链路中不要把 `/livox/lidar` 直接添加到 RViz 的 PointCloud2；应看 `/cloud_registered`。
也不要同时运行两种驱动启动文件，以免同名话题出现两种类型。

### 为什么新入口没有 `scout_base` 和 `pointcloud_to_laserscan`

新算法直接用雷达内置 IMU 和三维点云定位，建图不需要底盘轮式 `/odom`，也不需要先投影成二维 `/scan`。
默认入口不启动底盘驱动，手持遥控器仍直接控制车辆，保持手动档。
算法不会控制车辆运动。

### 为什么仍启动 `scout_description`

机器人模型负责 base_link、雷达安装位置、倾角等固定关系。
即使不启动底盘驱动，这些坐标关系仍然需要，才能在地图里正确放置车体与雷达。

## 5. `slam.launch.py` 的矩阵与 TF 解释

### 父子坐标关系是什么

`A → B` 表示在 A 坐标系里描述 B 的位置和方向。
平移不是唯一信息；旋转也会改变向前、向上的方向。
因此不能简单把两个 xyz 相减来替代完整变换。

本次 TF：

```text
map                         初始车体朝向，假设起步时位于平整地面
 └─ camera_init             FAST-LIO2 初始 IMU 坐标
     └─ body                FAST-LIO2 当前 IMU 坐标（动态定位结果）
         └─ base_link       四轮轴中心，静态安装换算
             ├─ base_footprint
             └─ livox_level
                 └─ livox_frame
```

`camera_init` 和 `body` 是上游代码沿用的名字，并不表示用了摄像头。
`body` 是 IMU 原点，不能直接当作车体原点。

### 两种“外参”不能混用

1. 车体到雷达：你测量的 `(0.130, 0, 0.326)` 和前倾 30°。
2. IMU 到雷达：雷达内部两个传感器原点之间的关系。

第二种使用本 ROS 2 移植版 Mid-360 配置给出的初值：

```yaml
extrinsic_T: [-0.011, -0.02329, 0.04412]
extrinsic_R: [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
```

这是雷达坐标在 IMU 坐标中的位姿；不是从 base_link 到雷达的实测距离。
本次沿用上游设备配置，未做你这台传感器的独立外参标定。
车头倾角不能再填进这个内部旋转里，否则会混淆安装关系。

### `mounting_transforms()` 的核心计算

```python
base_imu = base_lidar @ np.linalg.inv(imu_lidar)
```

数学含义：

```text
T_base_imu = T_base_lidar × inverse(T_imu_lidar)
```

`T` 是 4×4 矩阵，包含旋转和平移；`@` 是矩阵乘法；`inv()` 表示反向坐标换算。
已知车体到雷达、IMU 到雷达，就能算出车体到 IMU。
反过来即 `body → base_link`，把 FAST-LIO2 的 IMU 位姿连接到车体模型。

`Rotation.from_euler('xyz', [0, pitch, yaw])` 把 roll/pitch/yaw 变成旋转矩阵，使用与 Xacro 相同的 RPY 约定。
`as_quat()` 把旋转转为 ROS TF 使用的四元数 `[x,y,z,w]`。
这些都是同一姿态的不同表示，不是多加了一次旋转。

### 为什么还有 `map → camera_init`

这个上游实现初始旋转为单位矩阵，并把重力方向保留在估计状态里，初始世界坐标不一定自动水平。
你的雷达倾斜了约 30°，因此使用安装关系，把初始 IMU 坐标摆回初始车体参考方向。
该变换假设启动时车体在平地上，base_link 离地 0.079 m；不是在线测出的坡度。
若启动时在坡面上，这个初始水平假设不成立。

静态变换来自同一份外参，因此车体倾角只通过坐标链正确组合，不重复补偿。
`mapping.extrinsic_est_en=false` 固定内部外参；若打开在线外参估计，这个静态安装换算也需重新设计。

### 为什么不发布旧轮式 `odom → base_link`

新系统已经通过 `map → camera_init → body → base_link` 连接车体。
同时给 base_link 增加另一个父坐标系，会形成冲突。
因此启动 FAST-LIO2 前要关闭旧建图、旧底盘驱动和旧 TF 发布终端。
今后如果要融合轮式里程计，需要专门设计融合节点及 TF 发布权，不能直接叠加启动。

## 6. 参数文件每一组是什么意思

打开：`src/scout_mapping/config/fastlio_mid360.yaml`。

| 参数 | 当前值 | 用途 |
| --- | --- | --- |
| feature_extract_enable | false | 采用 FAST-LIO2 的直接点云匹配方式，不启用旧特征提取路径 |
| point_filter_num | 3 | 预处理抽样保留点，减少计算量 |
| max_iteration | 3 | 每次状态更新的最大迭代次数 |
| filter_size_surf | 0.15 m | 当前扫描降采样尺寸；越小通常点越密、计算越多 |
| filter_size_map | 0.20 m | 算法内部地图点降采样尺寸，不等同二维地图像素分辨率 |
| cube_side_length | 1000.0 m | 算法局部地图立方体尺寸，沿用上游初值；不是承诺的有效雷达距离 |
| common.lid_topic | /livox/lidar | 输入带逐点时间的雷达消息 |
| common.imu_topic | /livox/imu | 输入雷达内置 IMU |
| common.time_sync_en | false | 不开启上游简易软件时间校正，使用驱动已有时间戳 |
| common.time_offset_lidar_to_imu | 0.0 | 未独立标定额外时间偏移，初值为零 |
| preprocess.lidar_type | 1 | 使用 Livox CustomMsg 预处理 |
| preprocess.scan_line | 4 | 沿用上游 Mid-360 配置的 line 设置 |
| preprocess.blind | 0.5 m | 不使用距离太近的点，减少车体与近距离干扰；不是完整车体自过滤 |
| preprocess.timestamp_unit | 3 | 纳秒枚举；CustomMsg 的 offset_time 在专用处理函数里按纳秒转换 |
| preprocess.scan_rate | 10 | 对应当前驱动 10 Hz 发布频率 |
| mapping.acc_cov / gyr_cov | 0.1 / 0.1 | 加速度、角速度噪声模型初值，不是移动速度 |
| mapping.b_acc_cov / b_gyr_cov | 0.0001 / 0.0001 | IMU 偏置随机变化的噪声模型初值 |
| mapping.extrinsic_est_en | false | 固定内部外参，保持与 TF 换算一致 |
| mapping.fov_degree | 360° | 算法地图视场设置 |
| mapping.det_range | 100 m | 算法地图范围相关参数，沿用上游值，非本次实测性能 |
| publish.dense_publish_en | true | 输出去畸变后的较密扫描，RViz 可看到更多点 |
| publish.map_en | true | 打开累积 /Laser_map，保存服务依赖此地图缓存 |
| pcd_save.pcd_save_en | true | 允许保存 PCD |

这些是起步配置，还没有针对你的实际场景优化。更密不一定更准确，也不是盲目减小滤波尺寸就能消除发散。
该移植版的 /Laser_map 累积发布定时器约 1 Hz，累积的是定时取到的配准扫描；它不等于每帧完整高密度点的总和。
内部 ikd-Tree 用于定位匹配，和 /Laser_map 显示缓存的用途不同。
该显示缓存随建图时间增加，长时间建图要关注内存。
`pcd_save.interval` 在这个移植版的原生逐帧保存代码中未启用，不能依靠修改它实现分段保存。

## 7. 对上游 C++ 做了哪两处小修改

### 构建依赖

`CMakeLists.txt` 中移除了源码未使用的 `pcl_ros` 依赖，保留源码实际使用的 `pcl_conversions` 与 PCL。
显式补充 `tf2` 和 `tf2_ros`，避免依赖间接包含带来的构建问题。
`package.xml` 同步移除未使用的 pcl_ros，添加 tf2_ros。
算法的滤波、去畸变、点面匹配和 ikd-Tree 核心实现未重写。

### 地图保存结果检查

原代码不检查空点云和写文件结果，就回复保存成功。
本次修改后：

```cpp
if (pcl_wait_pub->empty()) return false;
return pcd_writer.writeBinary(map_file_path, *pcl_wait_pub) == 0;
```

- 没有地图点时返回失败。
- PCL 写文件成功才返回成功。
- 服务回调捕获写入异常，向用户返回错误消息。

所以服务调用后要看 `success: true`，不能只看命令是否执行。

## 8. 启动和使用

### 先停止旧系统

关闭旧的 mapping_system、scout_base、slam_toolbox、lidar_tf、单独 Livox 驱动终端，避免重复发布。
遥控器保持手动档。新建图入口不需要启动 can0，因为默认不与底盘 CAN 通信。

### 确认雷达网络

此前工作网口为 `enx00e04c10016b`；重新接线后先用 `ip -br addr` 确认网卡。
当前接收 IP 应为 192.168.1.5，雷达为 192.168.1.178。

```bash
ip route get 192.168.1.178
ping -I enx00e04c10016b -c 4 192.168.1.178
```

若重启后地址丢失，按原已验证方式恢复：

```bash
sudo ip link set enx00e04c10016b up
sudo ip addr replace 192.168.1.5/32 dev enx00e04c10016b
sudo ip route replace 192.168.1.178/32 dev enx00e04c10016b src 192.168.1.5
```

不要同时在断开的另一网卡上保留相同接收 IP。

### 一键启动

```bash
source /opt/ros/humble/setup.bash
source ~/scout_ws/install/setup.bash
ros2 launch scout_bringup mapping_system.launch.py
```

先让车在平地上静止数秒，等待 IMU 初始化完成和稳定点云输出，再手持遥控缓慢移动。
无需点击 2D Pose Estimate；本次是从启动位置开始的三维增量建图。

新 RViz 已设 Fixed Frame=map：
- Registered scan：当前配准扫描，Decay Time=0，便于检查当前帧。
- Accumulated map：/Laser_map，显示累积地图。
- Path：车辆 IMU 运动轨迹。

若只想检查当前帧厚度，可取消勾选 Accumulated map。

### 查看输入和输出

另开终端并 source 环境：

```bash
ros2 topic type /livox/lidar
# 应为 livox_ros_driver2/msg/CustomMsg
ros2 topic hz /livox/imu
# 确认持续有 IMU；按 Ctrl+C 后执行下一项
ros2 topic hz /cloud_registered
# 初始化后应持续有配准扫描输出
ros2 topic echo /Odometry --once
# header.frame_id=camera_init，child_frame_id=body
```

没有 /cloud_registered 时先查点云与 IMU 是否都持续输入，再看初始化、时间同步相关日志。
切换算法后，不要再用 `ros2 topic hz /scan` 作为新系统是否正常的判据。

### 保存地图

车停稳，建图节点保持运行：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

默认文件：`/home/robot/scout_ws/maps/fastlio_map.pcd`。
收到 `success: true` 才表示写入成功。同名文件会覆盖。
启动时可以换文件名：

```bash
ros2 launch scout_bringup mapping_system.launch.py \
  map_save_path:=/home/robot/scout_ws/maps/fastlio_lab_01.pcd
```

PCD 内保存的坐标是上游 camera_init 坐标，不包含 TF 中用于水平显示的 map→camera_init 变换。
因此直接用外部 PCD 查看器打开时，可能与 RViz 的 map 朝向不同；后续转二维导航地图前需应用该固定变换。
本次未实现 PCD→二维栅格图、已有 PCD 地图重定位、自动导航或 EGO-Planner。
PCD 也不是旧的 .pgm/.yaml 地图，不能直接交给现有 AMCL 地图服务。

### 切回旧二维方式

先 Ctrl+C 结束 FAST-LIO2 整套系统，再执行：

```bash
ros2 launch scout_bringup mapping_2d.launch.py
```

旧方式需要 can0、底盘里程计，并启动 PointCloud2 格式的雷达驱动。
两套系统不要同时启动。

## 9. 实际完成的验证与剩余检查

已完成：

1. FAST-LIO2 源码编译通过，scout_mapping / scout_bringup 安装完成。
2. 一键启动文件参数加载检查通过。
3. 在隔离 ROS_DOMAIN_ID=87 内，用合成静止点云和 IMU 验证整套新入口；没有连接底盘或真实雷达。
4. 收到 87 条 /Odometry、10 条 /Laser_map；map→base_link 可查询。
5. 合成静止测试的 base_link 高度约 0.07904 m，与设定 0.079 m 一致。
6. 空地图保存返回失败；非空地图保存成功，生成实际 PCD 文件。

未完成：真实 Mid-360 数据的时间同步、倾角方向、实际安装外参、车辆运动时的精度和稳定性验证。
合成测试只能证明软件链路可运行，不能代替实车效果。

## 10. 你发的原始点云截图应该怎么看

截图中 /livox/lidar 的彩色扫描有墙面形状，Fixed Frame=livox_level，Decay Time=0.2 秒。
这样的画面不能直接称为 FAST-LIO2 发散，因为当时没有运行这套 FAST-LIO2。
花纹、稀疏点和强度颜色也不等于定位误差。

新系统按这三个层次判断：

1. 当前配准扫描：普通墙面是不是稳定的薄平面，是否有明显反射异常点。
2. 累积地图：车静止时同一面墙是否持续变厚、长出多层。
3. 轨迹：车静止时 /Odometry 是否出现明显持续漂移，走一圈后墙体是否错位。

如果当前扫描也异常，先查原始观测和安装；只有累积后重影，重点查定位、同步与标定。
固定安装角度错误会造成整体朝向错误，但固定旋转本身不会把一个薄平面变成随机散点。

## 参考源码

- 原始 FAST-LIO2：https://github.com/hku-mars/FAST_LIO
- 本次 ROS 2 移植版：https://github.com/Ericsii/FAST_LIO_ROS2/tree/ros2
- 移植版 Mid-360 初始配置：https://github.com/Ericsii/FAST_LIO_ROS2/blob/ros2/config/mid360.yaml
- Livox 驱动：https://github.com/Livox-SDK/livox_ros_driver2
