/**
 * 传感器录制与回放 —— 底层就是标准的 rosbag2（MCAP 容器 + CDR + ros2msg schema）。
 *
 * 为什么不是自定义格式：录下来的包 `ros2 bag info` / `ros2 bag play` 直接能用，
 * Foxglove Studio 拖进去就能看点云和 IMU 曲线。多一个私有格式对谁都没好处。
 *
 * 两个用途：
 *   1. standalone 通信层的数据来源（不需要 ROS 就能回放）；
 *   2. 改滤波器时的回归对照 —— 同一段数据喂给新旧两版，比轨迹。
 *
 * 录的是「进入算法之前」的原始量，所以回放和实时跑的是同一条代码路径。
 *
 * 读的时候除了自己录的包，也能吃真机上用 `ros2 bag record` 录的包：
 * 每种雷达的 per-point 时间戳语义不一样（ns / 绝对秒 / 相对偏移 / 没有），
 * 靠配置里的 lidar_type 区分；自己录的包带 `t`(float64 绝对秒) 字段，会被自动识别。
 */

#pragma once

#include "common/common.h"

#include <pch.h>

namespace small_point_lio::transport {

    /// per-point 时间戳的语义。自己录的包不需要指定，会自动认出来。
    enum class PointTimeMode {
        /// 字段 `t`，float64，绝对秒。本项目录制时用的规范格式。
        CanonicalAbsoluteSeconds,
        /// 字段 `timestamp`，float64，纳秒（livox_ros_driver2 的 PointCloud2 输出）。
        LivoxNanoseconds,
        /// 字段 `timestamp`，float64，绝对秒（custom_mid360_driver）。
        AbsoluteSeconds,
        /// 字段 `timestamp`，float32，相对该帧 header 时间的偏移（unitree）。
        RelativeToHeader,
        /// 没有 per-point 时间，整帧用 header 时间。
        HeaderOnly,
    };

    /// 从配置里的 lidar_type 推出时间戳语义。认不出来时返回 HeaderOnly。
    PointTimeMode point_time_mode_from_lidar_type(const std::string &lidar_type);

    class RecordingWriter {
    public:
        RecordingWriter();
        ~RecordingWriter();

        RecordingWriter(const RecordingWriter &) = delete;
        RecordingWriter &operator=(const RecordingWriter &) = delete;

        bool open(const std::string &path,
                  const std::string &imu_topic,
                  const std::string &lidar_topic,
                  const std::string &frame_id,
                  std::string &error);
        bool is_open() const;
        void write_imu(const common::ImuMsg &imu_msg);
        void write_pointcloud(const std::vector<common::Point> &pointcloud);
        void close();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };

    class RecordingReader {
    public:
        RecordingReader();
        ~RecordingReader();

        RecordingReader(const RecordingReader &) = delete;
        RecordingReader &operator=(const RecordingReader &) = delete;

        bool open(const std::string &path,
                  const std::string &imu_topic,
                  const std::string &lidar_topic,
                  PointTimeMode fallback_mode,
                  std::string &error);
        void close();

        enum class Kind {
            Imu,
            Pointcloud,
            End,
        };

        /// 按时间顺序读下一条。返回 Kind::End 表示读完；
        /// 出错时把原因写进 error 并返回 End。
        Kind read_next(common::ImuMsg &imu_msg,
                       std::vector<common::Point> &pointcloud,
                       std::string &error);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };

}// namespace small_point_lio::transport
