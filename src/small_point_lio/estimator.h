/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "eskf.h"
#include "parameters.h"
#include "small_ivox.h"
#include <pch.h>

namespace small_point_lio {

    /// 运行期计数。加它是为了回答一个具体问题：IMU 饱和到底触发没有？
    /// eskf.h 里那个 satu_check 索引 bug 只在【加速度计饱和】时才咬人，
    /// 如果一段数据里 acc_saturated 全是 0，那这个修复在这段数据上就是空操作 ——
    /// 这是判断「修复有没有意义」的证据，不能靠猜。
    struct Diagnostics {
        uint64_t imu_updates = 0;
        uint64_t gyro_saturated[3] = {0, 0, 0};
        uint64_t acc_saturated[3] = {0, 0, 0};

        uint64_t gyro_saturated_total() const {
            return gyro_saturated[0] + gyro_saturated[1] + gyro_saturated[2];
        }
        uint64_t acc_saturated_total() const {
            return acc_saturated[0] + acc_saturated[1] + acc_saturated[2];
        }
    };

    class Estimator {
    public:
        // for common
        Parameters *parameters = nullptr;
        eskf kf;
        // for h_point
        std::shared_ptr<SmallIVox> ivox;
        Eigen::Matrix<state::value_type, 3, 1> Lidar_T_wrt_IMU;
        Eigen::Matrix<state::value_type, 3, 3> Lidar_R_wrt_IMU;
        Eigen::Vector3f point_lidar_frame;
        Eigen::Vector3f point_odom_frame;
        std::vector<Eigen::Vector3f> nearest_points;
        // for h_imu
        Eigen::Matrix<state::value_type, 3, 1> angular_velocity;
        Eigen::Matrix<state::value_type, 3, 1> linear_acceleration;
        double imu_acceleration_scale;
        Diagnostics diagnostics;

        Estimator();

        void reset();

        [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> process_noise_cov() const;

        void h_point(const state &s, point_measurement_result &measurement_result);

        void h_imu(const state &s, imu_measurement_result &measurement_result);
    };

}// namespace small_point_lio
