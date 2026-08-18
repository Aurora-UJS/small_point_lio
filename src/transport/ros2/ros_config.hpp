/**
 * ROS 侧的参数读取。
 *
 * 这里刻意保留 declare_parameter 这条路，而不是让 ROS 也去读 yaml 文件：
 * 保住 ros2 param set / launch 参数覆盖 / 参数热重载这一整套生态。
 * 两条路最后填的是同一个 Parameters 结构体。
 */

#pragma once

#include "small_point_lio/parameters.h"
#include "transport/transport.h"

#include <rclcpp/rclcpp.hpp>

namespace small_point_lio::transport {

    void read_parameters_from_node(rclcpp::Node &node,
                                   Parameters &parameters,
                                   TransportConfig &transport_config);

}// namespace small_point_lio::transport
