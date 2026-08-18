/**
 * 算法核心的日志出口。
 *
 * 核心代码不认识 rclcpp，也不该认识。它只把日志丢给一个可替换的 sink：
 *   - ROS 通信层把 sink 接到 RCLCPP_*，日志照旧进 /rosout；
 *   - standalone 通信层用默认 sink，直接打到 stderr。
 */

#pragma once

#include <pch.h>

namespace common {

    enum class LogLevel {
        Debug,
        Info,
        Warn,
        Error,
    };

    using LogSink = std::function<void(LogLevel, const std::string &)>;

    /// 替换日志出口。传空则恢复默认的 stderr 输出。
    void set_log_sink(LogSink sink);

    void log(LogLevel level, const std::string &message);

}// namespace common

#define SPL_LOG_DEBUG(msg) ::common::log(::common::LogLevel::Debug, (msg))
#define SPL_LOG_INFO(msg) ::common::log(::common::LogLevel::Info, (msg))
#define SPL_LOG_WARN(msg) ::common::log(::common::LogLevel::Warn, (msg))
#define SPL_LOG_ERROR(msg) ::common::log(::common::LogLevel::Error, (msg))
