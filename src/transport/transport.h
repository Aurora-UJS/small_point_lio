/**
 * 通信层（transport）接口。
 *
 * 算法核心只认两件事：把 common::Point / common::ImuMsg 喂进去，把 common::Odometry
 * 和点云拿出来。至于这些数据是从 ROS 话题来的、从共享内存来的、还是从录制文件回放的，
 * 核心一概不知道。
 *
 * 一个 transport 实现负责三件事：
 *   1. 建立数据来源（订阅话题 / 打开设备 / 打开录制文件）；
 *   2. 把算法输出送出去（发布话题 / 写文件 / 推给别的进程）；
 *   3. 驱动主循环直到退出。
 *
 * 编译期用 SPL_WITH_ROS2 / SPL_WITH_STANDALONE 决定把哪些实现编进来；
 * 运行期用配置文件里的 transport 字段决定用哪一个。
 * 在没有 ROS 的板子上只编 standalone，产出的二进制完全不链接 rclcpp。
 */

#pragma once

#include "small_point_lio/parameters.h"

#include <pch.h>

namespace small_point_lio::transport {

    /// 算法之外、属于通信层的配置。
    struct TransportConfig {
        /// "ros2" 或 "standalone"。配置文件里改这一个字段就能换通信层。
        std::string kind = "ros2";

        // ---- ROS 通信层 ----
        std::string lidar_topic = "/livox/lidar";
        std::string imu_topic = "/livox/imu";
        /// livox_custom_msg / livox_pointcloud2 / custom_mid360_driver / unilidar / standard_pointcloud2
        std::string lidar_type = "livox_custom_msg";
        std::string lidar_frame = "livox_frame";
        bool save_pcd = false;

        /// 非空时，ROS 通信层把收到的原始点云/IMU 原样录进这个文件，
        /// 供 standalone 通信层离线回放。这是做算法回归对比用的。
        std::string record_path;

        // ---- standalone 通信层 ----
        /// 回放用的录制文件路径。
        std::string replay_path;
        /// 里程计输出落盘路径（TUM 格式：timestamp tx ty tz qx qy qz qw）。空则只打到 stdout。
        std::string odometry_output_path;
        /// 回放结束后把建好的点云图存成 pcd 的路径。空则不存。
        std::string pcd_output_path;
    };

    class ITransport {
    public:
        virtual ~ITransport() = default;

        /// 建立数据来源与输出通道。失败返回 false。
        virtual bool start() = 0;

        /// 阻塞运行直到数据跑完或收到退出信号。返回进程退出码。
        virtual int spin() = 0;

        virtual void stop() = 0;
    };

    /// 按 transport_config.kind 造出对应实现。
    /// 若该实现没有被编译进来，返回 nullptr 并把原因写进 error。
    std::unique_ptr<ITransport> make_transport(const TransportConfig &transport_config,
                                               const Parameters &parameters,
                                               std::string &error);

    /// 本次构建实际编进来的 transport 名字，供报错时提示用。
    std::vector<std::string> available_transports();

}// namespace small_point_lio::transport
