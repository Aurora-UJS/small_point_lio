/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

/**
 * ROS 2 通信层。
 *
 * 两种用法，行为一致：
 *   1. 作为 rclcpp 组件 / small_point_lio_node 可执行文件 —— 参数走 ROS 参数系统，
 *      ros2 param set、launch 覆盖、参数热重载都照旧；
 *   2. 由统一入口 small_point_lio_standalone 按配置文件里的 transport: ros2 拉起 ——
 *      参数走 yaml 解析器，其余完全相同。
 */

#pragma once

#include "common/common.h"
#include "small_point_lio/small_point_lio.h"
// 录制依赖 mcap。SPL_WITH_MCAP 由 spl_recording 这个 target PUBLIC 定义，
// 只有真正链接了它才成立 —— 保证「用到录制」和「链接得到录制」始终一致。
#ifdef SPL_WITH_MCAP
#include "transport/recording.h"
#endif
#include "transport/ros2/lidar_adapter/base_lidar.h"
#include "transport/transport.h"
#include "util/pointcloud_mapping.h"

#include <nav_msgs/msg/odometry.hpp>
#include <pch.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>

namespace small_point_lio {

    class SmallPointLioNode : public rclcpp::Node {
    private:
        std::unique_ptr<small_point_lio::SmallPointLio> small_point_lio;
        std::vector<common::Point> pointcloud;
        std::unique_ptr<LidarAdapterBase> lidar_adapter;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subsciber;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
        common::Odometry last_odometry;
        std::unique_ptr<util::PointcloudMapping> pointcloud_mapping;
#ifdef SPL_WITH_MCAP
        std::unique_ptr<transport::RecordingWriter> recording_writer;
        std::mutex recording_mutex;
#endif
        /// initialize() 是否成功走完。失败时 start() 必须返回 false，
        /// 否则 spin() 会对已经 shutdown 的 context 调 rclcpp::spin 而崩溃。
        bool initialized = false;

        /// 两个构造入口的共同部分：建发布/订阅、接回调、装雷达适配器。
        void initialize(const Parameters &parameters,
                        const transport::TransportConfig &transport_config);

    public:
        /// 组件入口：参数从 ROS 参数系统读。
        explicit SmallPointLioNode(const rclcpp::NodeOptions &options);

        /// 统一入口：参数已经由调用方（yaml 解析器）准备好。
        SmallPointLioNode(const rclcpp::NodeOptions &options,
                          const Parameters &parameters,
                          const transport::TransportConfig &transport_config);

        [[nodiscard]] bool is_initialized() const { return initialized; }
    };

    namespace transport {

        class RosTransport : public ITransport {
        public:
            RosTransport(const TransportConfig &transport_config, const Parameters &parameters);
            ~RosTransport() override;

            RosTransport(const RosTransport &) = delete;
            RosTransport &operator=(const RosTransport &) = delete;

            bool start() override;
            int spin() override;
            void stop() override;

        private:
            TransportConfig transport_config;
            Parameters parameters;
            std::shared_ptr<SmallPointLioNode> node;
            bool owns_context = false;
        };

    }// namespace transport

}// namespace small_point_lio
