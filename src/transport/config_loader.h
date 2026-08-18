/**
 * 配置读取（不依赖 ROS）。
 *
 * 解析的是和 ROS 参数文件同一份 yaml —— 只支持这份文件实际用到的子集：
 * 一层 `小节: / ros__parameters:` 之下的平铺 `key: 标量` 与 `key: [列表]`
 * （列表允许换行），`#` 之后是注释。
 *
 * 这样做的意义：真机和仿真、ROS 和非 ROS 用的是同一个配置文件，
 * 不会出现「改了 yaml 忘了改 toml」这种两份配置飘掉的问题。
 */

#pragma once

#include "small_point_lio/parameters.h"
#include "transport/transport.h"

#include <pch.h>

namespace small_point_lio::transport {

    /// 从 yaml 读出算法参数与通信层配置。失败返回 false，原因写进 error。
    bool load_config_file(const std::string &path,
                          Parameters &parameters,
                          TransportConfig &transport_config,
                          std::string &error);

}// namespace small_point_lio::transport
