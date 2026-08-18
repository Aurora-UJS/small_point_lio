/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "parameters.h"

namespace small_point_lio {

    void Parameters::finalize(double min_distance, double max_distance,
                              double raw_satu_acc, double raw_satu_gyro) {
        min_distance_squared = static_cast<float>(min_distance * min_distance);
        max_distance_squared = static_cast<float>(max_distance * max_distance);
        // 留 1% 余量：传感器报的饱和值本身有量化误差，卡在等号上会漏判。
        satu_acc = raw_satu_acc * 0.99;
        satu_gyro = raw_satu_gyro * 0.99;
    }

}// namespace small_point_lio
