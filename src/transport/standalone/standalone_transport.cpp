#include "transport/standalone/standalone_transport.h"

#include "common/log.h"
#include "io/pcd_io.h"
#include "transport/recording.h"

#include <cstdio>

namespace small_point_lio::transport {

    StandaloneTransport::StandaloneTransport(const TransportConfig &transport_config,
                                             const Parameters &parameters)
        : transport_config(transport_config), parameters(parameters) {}

    bool StandaloneTransport::start() {
        if (transport_config.replay_path.empty()) {
            SPL_LOG_ERROR("standalone 通信层需要 replay_path，配置文件里没有给");
            return false;
        }
        small_point_lio = std::make_unique<SmallPointLio>(parameters);

        if (transport_config.save_pcd || !transport_config.pcd_output_path.empty()) {
            pointcloud_mapping = std::make_unique<util::PointcloudMapping>(0.02);
        }
        if (!transport_config.odometry_output_path.empty()) {
            odometry_output = std::fopen(transport_config.odometry_output_path.c_str(), "w");
            if (odometry_output == nullptr) {
                SPL_LOG_ERROR("无法写入轨迹文件: " + transport_config.odometry_output_path);
                return false;
            }
            // TUM 格式：timestamp tx ty tz qx qy qz qw
            std::fprintf(odometry_output, "# timestamp tx ty tz qx qy qz qw\n");
        }

        small_point_lio->set_odometry_callback([this](const common::Odometry &odometry) {
            ++odometry_count;
            if (odometry_output != nullptr) {
                std::fprintf(odometry_output, "%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n",
                             odometry.timestamp,
                             odometry.position.x(), odometry.position.y(), odometry.position.z(),
                             odometry.orientation.x(), odometry.orientation.y(),
                             odometry.orientation.z(), odometry.orientation.w());
            }
        });
        small_point_lio->set_pointcloud_callback([this](const std::vector<Eigen::Vector3f> &pointcloud) {
            if (pointcloud_mapping) {
                for (const auto &point: pointcloud) {
                    pointcloud_mapping->add_point(point);
                }
            }
        });
        running = true;
        return true;
    }

    int StandaloneTransport::spin() {
        RecordingReader reader;
        std::string error;
        // 自己录的包带 `t`(float64 绝对秒) 字段会被自动识别；
        // 真机上 ros2 bag record 录的包按 lidar_type 决定时间戳语义。
        PointTimeMode fallback_mode = point_time_mode_from_lidar_type(transport_config.lidar_type);
        if (!reader.open(transport_config.replay_path,
                         transport_config.imu_topic,
                         transport_config.lidar_topic,
                         fallback_mode,
                         error)) {
            SPL_LOG_ERROR(error);
            return 1;
        }
        SPL_LOG_INFO("开始回放: " + transport_config.replay_path);

        common::ImuMsg imu_msg;
        std::vector<common::Point> pointcloud;
        size_t imu_count = 0;
        size_t frame_count = 0;
        while (running) {
            RecordingReader::Kind kind = reader.read_next(imu_msg, pointcloud, error);
            if (kind == RecordingReader::Kind::End) {
                break;
            }
            if (kind == RecordingReader::Kind::Imu) {
                small_point_lio->on_imu_callback(imu_msg);
                ++imu_count;
            } else {
                small_point_lio->on_point_cloud_callback(pointcloud);
                ++frame_count;
            }
            // 和 ROS 通信层保持同一条驱动路径：每收到一份数据就推一次状态机。
            small_point_lio->handle_once();
        }
        if (!error.empty()) {
            SPL_LOG_ERROR(error);
        }
        SPL_LOG_INFO("回放结束：IMU " + std::to_string(imu_count) +
                     " 条，点云 " + std::to_string(frame_count) +
                     " 帧，里程计输出 " + std::to_string(odometry_count) + " 次");

        // 饱和统计：acc_saturated 全 0 意味着 eskf 那个 satu_check 索引修复
        // 在这段数据上根本不会被触发，两版轨迹理应完全一致。
        const Diagnostics &diagnostics = small_point_lio->diagnostics();
        SPL_LOG_INFO("IMU 更新 " + std::to_string(diagnostics.imu_updates) +
                     " 次；陀螺饱和 " + std::to_string(diagnostics.gyro_saturated_total()) +
                     " 次 (xyz " + std::to_string(diagnostics.gyro_saturated[0]) + "/" +
                     std::to_string(diagnostics.gyro_saturated[1]) + "/" +
                     std::to_string(diagnostics.gyro_saturated[2]) +
                     ")；加速度饱和 " + std::to_string(diagnostics.acc_saturated_total()) +
                     " 次 (xyz " + std::to_string(diagnostics.acc_saturated[0]) + "/" +
                     std::to_string(diagnostics.acc_saturated[1]) + "/" +
                     std::to_string(diagnostics.acc_saturated[2]) + ")");
        if (diagnostics.acc_saturated_total() == 0) {
            SPL_LOG_WARN("这段数据里加速度计从未饱和 —— eskf satu_check 索引修复对它是空操作，"
                         "两版轨迹应当完全一致。要验证该修复，需要一段真的会撞到 satu_acc 阈值的数据。");
        }

        if (pointcloud_mapping && !transport_config.pcd_output_path.empty()) {
            std::vector<Eigen::Vector3f> points = pointcloud_mapping->get_points();
            io::pcd::write_pcd(transport_config.pcd_output_path, points);
            SPL_LOG_INFO("点云图已存到 " + transport_config.pcd_output_path);
        }
        stop();
        return error.empty() ? 0 : 1;
    }

    void StandaloneTransport::stop() {
        running = false;
        if (odometry_output != nullptr) {
            std::fclose(odometry_output);
            odometry_output = nullptr;
        }
    }

}// namespace small_point_lio::transport
