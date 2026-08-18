# Small Point-LIO

Small Point-LIO is an advanced implementation of the [Point-LIO algorithm](https://github.com/hku-mars/Point-LIO), delivering a 2-3x speed improvement over the original.

The default branch is for ROS2. If you want to run it without ros, please checkout the `main` branch.

If you want to know why it so fast, please read [this](https://bbs.robomaster.com/article/813022).

<img src="./img/ACE.jpg" width="200px">

## 通信层（Aurora-UJS 分支新增）

算法核心（`spl_core`）不认识 ROS，ROS 只是外面套的一层壳。同一份代码可以编成两种形态：

| 构建方式 | 产物 | 依赖 |
| --- | --- | --- |
| `colcon build`（默认，`SPL_WITH_ROS2=ON`） | `small_point_lio_node`（ROS 组件）+ `small_point_lio_standalone`（统一入口） | ROS 2 Jazzy |
| `cmake -DSPL_WITH_ROS2=OFF` | `small_point_lio_standalone` | 只有 libstdc++ / libm / libgomp |

第二种产物 `ldd` 出来只有 `libc`、`libgcc_s`、`libgomp`、`libm`、`libstdc++`，可以直接交叉编译到没有 ROS 的
32 位 ARM 板子上。

### 换通信层

编进来多个通信层时，运行期由配置文件里的 `transport` 字段决定用哪个：

```yaml
small_point_lio:
    ros__parameters:
        transport: ros2        # 或 standalone
```

`transport: ros2` 时行为和原来完全一致（发 `/Odometry`、`/cloud_registered`、tf）。
`transport: standalone` 时从 `replay_path` 指定的 rosbag2（MCAP）回放，把轨迹写成 TUM 格式。

原来的 `ros2 launch` / `ros2 run small_point_lio small_point_lio_node` 那条路没有变化，
参数照旧走 ROS 参数系统（`ros2 param set` 仍然可用）。

### 录制与离线回放

改滤波器时需要一个回归对照：同一段数据喂给新旧两版，比轨迹。为此 ROS 通信层可以把
**进入算法之前**的原始点云和 IMU 原样落盘：

```yaml
record_path: "/path/to/run_0.mcap"     # 非空即开始录制
```

录出来的是**标准 rosbag2**（MCAP 容器 + CDR + ros2msg schema，zstd 压缩），不是私有格式：

```bash
ros2 bag info -s mcap run_0.mcap       # 能看
ros2 bag play -s mcap run_0.mcap       # 能放
```
Foxglove Studio 直接拖进去也能看点云和 IMU 曲线。

然后用 standalone 离线跑，走的是和实时完全相同的数据路径：

```bash
small_point_lio_standalone config/mid360_real.yaml   # transport: standalone, replay_path 指向 bag
```

回放也能吃真机上用 `ros2 bag record` 录的包。每种雷达的 per-point 时间戳语义不同
（ns / 绝对秒 / 相对偏移 / 没有），靠配置里的 `lidar_type` 区分；本项目自己录的包带
`t`(float64 绝对秒) 字段，会被自动识别，不受 `lidar_type` 影响。

### 回归对比

`tools/compare_runs.py` 把同一段录制喂给两个版本，逐时刻对齐后比轨迹并出图：

```bash
# 比两个 git ref（各自在临时 worktree 里构建）
tools/compare_runs.py --bag run_0.mcap --config config/mid360_real.yaml \
                      --ref ros2 --ref fix/eskf-satu-acc-index

# 或者比两个已有的可执行文件
tools/compare_runs.py --bag run_0.mcap --config config/mid360_real.yaml \
                      --binary A=/path/a --binary B=/path/b
```

它同时会报告 IMU 饱和次数。这一项很关键：`eskf.h` 里的 `satu_check` 索引问题
**只在加速度计饱和时才起作用**，如果一段数据里加速度饱和 0 次，那两版轨迹必然相同，
比出「无差异」并不代表修复没用，只代表这段数据测不到它。

### 配置文件

`config/mid360.yaml` 已拆成两份，因为真机和仿真的 IMU 单位不同，混用会直接发散：

| 文件 | 场景 | `acc_norm` | `satu_acc` |
| --- | --- | --- | --- |
| `config/mid360_real.yaml` | 真 Mid-360（Livox SDK 输出 g） | 1.0 | 3.0 |
| `config/mid360_sim.yaml` | Gazebo 仿真（输出 m/s²） | 9.81 | 30.0 |

代码里 `imu_acceleration_scale = |gravity| / acc_norm`，配错这一项等于把重力放大或缩小 9.81 倍。

## Contact

QQ group: 1070252119

Email: 1709185482@qq.com

## Param

here are some parameters you can set in config file:

```yaml
small_point_lio:
    ros__parameters:
        lidar_topic: /livox/lidar # LiDAR topic name
        imu_topic: /livox/imu # IMU topic name
        lidar_type: livox # Lidar type
        lidar_frame: livox_frame # Lidar frame
        save_pcd: false # Whether to save point cloud

        # Point Cloud Filtering
        point_filter_num: 1 # keep one point every N points
        min_distance: 0.5 # Minimum point cloud radius; points closer than this will be filtered
        max_distance: 1000 # Maximum point cloud radius; points farther than this will be filtered
        space_downsample: true # Whether to enable point cloud downsampling
        space_downsample_leaf_size: 0.5 # Voxel size used for point cloud downsampling

        # IMU Processing
        gravity: [0.0, 0.0, -9.810] # Gravity vector
        fix_gravity_direction: true # Whether to use the first 200 IMU data points to correct gravity direction (magnitude still from gravity parameter)
        check_satu: true # Whether to enable IMU data saturation check
        satu_acc: 3.0 # IMU acceleration saturation threshold
        satu_gyro: 35.0 # IMU angular velocity saturation threshold
        acc_norm: 1.0 # IMU acceleration norm

        # Map
        map_resolution: 0.5 # Map resolution
        init_map_size: 10 # Number of points required to initialize the map

        # LiDAR-IMU Extrinsic Calibration
        extrinsic_est_en: false # Whether to estimate LiDAR-IMU extrinsic transformation online
        extrinsic_T: [-0.011, -0.02329, 0.04412]
        extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Kalman Filter Parameters
        # R
        laser_point_cov: 0.01 # Laser point covariance
        imu_meas_acc_cov: 0.01 # IMU measured acceleration covariance
        imu_meas_omg_cov: 0.01 # IMU measured angular velocity covariance
        # Q
        velocity_cov: 20.0 # Velocity covariance
        acceleration_cov: 500.0 # Acceleration covariance
        omg_cov: 1000.0 # Angular velocity covariance
        ba_cov: 0.0001 # Acceleration bias covariance
        bg_cov: 0.0001 # Gyroscope bias covariance
        plane_thr: 0.1 # Plane matching threshold (smaller value = stricter)
        match_s: 81.0 # Point-to-plane association threshold (smaller value = stricter)

        # Data Publishing
        publish_odometry_without_downsample: false # Whether to publish high-frequency odometry. Note that this does not enhance the real-time nature of the odometry and but degrades performance. It is recommended to increase the point cloud publishing rate to achieve highly real-time odometry.
```

## Save map

**Step 1**: set `save_pcd` to `true` in config file.

**Step 2**: run small point lio until the map is finished.

**Step 3**: save map by calling service:

```cpp
ros2 service call /map_save std_srvs/srv/Trigger
```

> Note: Please make sure you have enough memery to save map. Don't forget to set `save_pcd` to `false` after saving.

## Third-party

Small Point-LIO is built in on and with the aid of the following open source projects. Credits are given to these projects.

|                            project                             |                          description                           |                                            license                                            |
| :------------------------------------------------------------: | :------------------------------------------------------------: | :-------------------------------------------------------------------------------------------: |
|           [Eigen](https://gitlab.com/libeigen/eigen)           |           A C++ template library for linear algebra            | [Mozilla Public License Version 2.0](https://gitlab.com/libeigen/eigen/-/blob/master/LICENSE) |
| [unordered_dense](https://github.com/martinus/unordered_dense) |          A fast & densely stored hashmap and hashset           |         [MIT License](https://github.com/martinus/unordered_dense/blob/main/LICENSE)          |
|       [small_gicp](https://github.com/koide3/small_gicp)       | Efficient and parallel algorithms for point cloud registration |            [MIT License](https://github.com/koide3/small_gicp/blob/master/LICENSE)            |
|          [Open3D](https://github.com/isl-org/Open3D)           |            A Modern Library for 3D Data Processing             |                  [MIT License](github.com/isl-org/Open3D/blob/main/LICENSE)                   |

## License

Copyright (C) 2025 Yingjie Huang

Licensed under the MIT License. See License.txt in the project root for license information.
