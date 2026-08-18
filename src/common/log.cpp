#include "log.h"

#include <cstdio>
#include <mutex>

namespace common {

    namespace {

        std::mutex &sink_mutex() {
            static std::mutex mutex;
            return mutex;
        }

        LogSink &sink_storage() {
            static LogSink sink;
            return sink;
        }

        const char *level_name(LogLevel level) {
            switch (level) {
                case LogLevel::Debug:
                    return "DEBUG";
                case LogLevel::Info:
                    return "INFO";
                case LogLevel::Warn:
                    return "WARN";
                case LogLevel::Error:
                    return "ERROR";
            }
            return "INFO";
        }

    }// namespace

    void set_log_sink(LogSink sink) {
        std::lock_guard<std::mutex> guard(sink_mutex());
        sink_storage() = std::move(sink);
    }

    void log(LogLevel level, const std::string &message) {
        LogSink sink;
        {
            std::lock_guard<std::mutex> guard(sink_mutex());
            sink = sink_storage();
        }
        if (sink) {
            sink(level, message);
            return;
        }
        std::fprintf(stderr, "[%s] [small_point_lio] %s\n", level_name(level), message.c_str());
    }

}// namespace common
