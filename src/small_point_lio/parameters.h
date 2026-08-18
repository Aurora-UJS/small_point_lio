/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace small_point_lio {

    class Parameters {
    public:
        int point_filter_num;
        float min_distance_squared;
        float max_distance_squared;
        bool space_downsample;
        float space_downsample_leaf_size;

        Eigen::Vector3d gravity;
        bool check_satu;
        bool fix_gravity_direction;
        double satu_acc;
        double satu_gyro;
        double acc_norm;

        double map_resolution;
        size_t init_map_size;

        bool extrinsic_est_en;
        Eigen::Vector3d extrinsic_T;
        Eigen::Matrix3d extrinsic_R;

        double laser_point_cov;
        double imu_meas_acc_cov;
        double imu_meas_omg_cov;
        double velocity_cov;
        double omg_cov;
        double acceleration_cov;
        double bg_cov;
        double ba_cov;
        double plane_threshold;
        double match_sqaured;

        bool publish_odometry_without_downsample = false;

        /// 由 min_distance / max_distance 换算平方值，并对 satu 阈值留 1% 余量。
        /// 各个通信层读完原始配置后必须调用一次，否则过滤和饱和检查行为不对。
        void finalize(double min_distance, double max_distance,
                      double raw_satu_acc, double raw_satu_gyro);
    };

}// namespace small_point_lio
