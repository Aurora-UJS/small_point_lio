/**
 * 非 ROS 通信层。
 *
 * 数据来源是 .splrec 录制文件；输出是 TUM 格式轨迹文件（+ 可选 pcd 点云图）。
 * 整条链路只用 C++ 标准库和 Eigen，可以在没有 ROS 的机器、甚至交叉编译到
 * 32 位 ARM 的板子上跑。
 *
 * 真机直连（Livox SDK2 / 共享内存）留在 `TODO(livox-sdk)` 那个位置接入：
 * 只要往 SmallPointLio::on_point_cloud_callback / on_imu_callback 喂数据即可，
 * 和这里回放文件走的是同一条路径。
 */

#pragma once

#include "small_point_lio/small_point_lio.h"
#include "transport/transport.h"
#include "util/pointcloud_mapping.h"

#include <pch.h>

namespace small_point_lio::transport {

    class StandaloneTransport : public ITransport {
    public:
        StandaloneTransport(const TransportConfig &transport_config, const Parameters &parameters);

        bool start() override;
        int spin() override;
        void stop() override;

    private:
        TransportConfig transport_config;
        Parameters parameters;
        std::unique_ptr<SmallPointLio> small_point_lio;
        std::unique_ptr<util::PointcloudMapping> pointcloud_mapping;
        std::FILE *odometry_output = nullptr;
        size_t odometry_count = 0;
        bool running = false;
    };

}// namespace small_point_lio::transport
