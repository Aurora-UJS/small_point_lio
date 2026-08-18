#pragma once

#ifndef PCH_H
#define PCH_H

// 一些参数
#include <param_deliver.h>
// STL
#define _USE_MATH_DEFINES
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <string>
#include <vector>
// Eigen
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
// omp
#include <omp.h>
// ankerl
#include <ankerl/unordered_dense.h>
// liblzf
#include <liblzf/lzf.h>

// 这里刻意不再包含 rclcpp。
// 算法核心（src/common, src/io, src/small_point_lio, src/util）必须能在没有 ROS 的
// 环境下编译。ROS 头文件只允许出现在 src/transport/ros2/ 下面。

#endif// PCH_H
