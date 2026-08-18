#include "transport/config_loader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace small_point_lio::transport {

    namespace {

        std::string strip_comment(const std::string &line) {
            bool in_single = false;
            bool in_double = false;
            for (size_t i = 0; i < line.size(); ++i) {
                char c = line[i];
                if (c == '\'' && !in_double) {
                    in_single = !in_single;
                } else if (c == '"' && !in_single) {
                    in_double = !in_double;
                } else if (c == '#' && !in_single && !in_double) {
                    return line.substr(0, i);
                }
            }
            return line;
        }

        std::string trim(const std::string &text) {
            size_t begin = text.find_first_not_of(" \t\r\n");
            if (begin == std::string::npos) {
                return {};
            }
            size_t end = text.find_last_not_of(" \t\r\n");
            return text.substr(begin, end - begin + 1);
        }

        std::string unquote(const std::string &text) {
            if (text.size() >= 2 &&
                ((text.front() == '"' && text.back() == '"') ||
                 (text.front() == '\'' && text.back() == '\''))) {
                return text.substr(1, text.size() - 2);
            }
            return text;
        }

        using KeyValues = ankerl::unordered_dense::map<std::string, std::string>;

        /// 把 yaml 拍平成 key -> 原始值文本。列表保留成 "[a, b, c]"。
        bool flatten(const std::string &path, KeyValues &out, std::string &error) {
            std::ifstream input(path);
            if (!input) {
                error = "打不开配置文件: " + path;
                return false;
            }
            bool in_parameters = false;
            std::string pending_key;
            std::string pending_value;
            std::string raw_line;
            while (std::getline(input, raw_line)) {
                std::string line = strip_comment(raw_line);
                std::string content = trim(line);
                if (content.empty()) {
                    continue;
                }
                // 正在拼一个跨行的列表
                if (!pending_key.empty()) {
                    pending_value += " " + content;
                    if (content.find(']') != std::string::npos) {
                        out[pending_key] = trim(pending_value);
                        pending_key.clear();
                        pending_value.clear();
                    }
                    continue;
                }
                if (content == "ros__parameters:") {
                    in_parameters = true;
                    continue;
                }
                size_t colon = content.find(':');
                if (colon == std::string::npos) {
                    continue;
                }
                std::string key = trim(content.substr(0, colon));
                std::string value = trim(content.substr(colon + 1));
                if (value.empty()) {
                    // 是个小节标题（例如最外层的 small_point_lio:），跳过
                    continue;
                }
                if (!in_parameters) {
                    continue;
                }
                if (value.front() == '[' && value.find(']') == std::string::npos) {
                    pending_key = key;
                    pending_value = value;
                    continue;
                }
                out[key] = value;
            }
            if (!pending_key.empty()) {
                error = "配置文件里的列表没有闭合: " + pending_key;
                return false;
            }
            if (!in_parameters) {
                error = "配置文件里找不到 ros__parameters 小节: " + path;
                return false;
            }
            return true;
        }

        class Reader {
        public:
            Reader(const KeyValues &values, std::string &error)
                : values(values), error(error) {}

            bool ok() const { return error.empty(); }

            std::string string_of(const std::string &key, const std::string &fallback) {
                auto it = values.find(key);
                if (it == values.end()) {
                    return fallback;
                }
                return unquote(it->second);
            }

            std::string required_string(const std::string &key) {
                auto it = values.find(key);
                if (it == values.end()) {
                    fail(key, "缺少该配置项");
                    return {};
                }
                return unquote(it->second);
            }

            bool boolean(const std::string &key, bool fallback, bool required) {
                auto it = values.find(key);
                if (it == values.end()) {
                    if (required) {
                        fail(key, "缺少该配置项");
                    }
                    return fallback;
                }
                std::string text = unquote(it->second);
                if (text == "true" || text == "True" || text == "1") {
                    return true;
                }
                if (text == "false" || text == "False" || text == "0") {
                    return false;
                }
                fail(key, "不是合法的布尔值: " + text);
                return fallback;
            }

            double number(const std::string &key) {
                auto it = values.find(key);
                if (it == values.end()) {
                    fail(key, "缺少该配置项");
                    return 0.0;
                }
                try {
                    size_t consumed = 0;
                    std::string text = unquote(it->second);
                    double parsed = std::stod(text, &consumed);
                    if (trim(text.substr(consumed)).empty()) {
                        return parsed;
                    }
                } catch (const std::exception &) {
                }
                fail(key, "不是合法的数值: " + it->second);
                return 0.0;
            }

            std::vector<double> numbers(const std::string &key, size_t expected) {
                std::vector<double> result;
                auto it = values.find(key);
                if (it == values.end()) {
                    fail(key, "缺少该配置项");
                    return std::vector<double>(expected, 0.0);
                }
                std::string text = it->second;
                size_t open = text.find('[');
                size_t close = text.rfind(']');
                if (open == std::string::npos || close == std::string::npos || close < open) {
                    fail(key, "不是合法的列表: " + text);
                    return std::vector<double>(expected, 0.0);
                }
                std::stringstream stream(text.substr(open + 1, close - open - 1));
                std::string item;
                while (std::getline(stream, item, ',')) {
                    std::string trimmed = trim(item);
                    if (trimmed.empty()) {
                        continue;
                    }
                    try {
                        result.push_back(std::stod(trimmed));
                    } catch (const std::exception &) {
                        fail(key, "列表里有非数值项: " + trimmed);
                        return std::vector<double>(expected, 0.0);
                    }
                }
                if (result.size() != expected) {
                    fail(key, "列表长度应为 " + std::to_string(expected) +
                                      "，实际为 " + std::to_string(result.size()));
                    return std::vector<double>(expected, 0.0);
                }
                return result;
            }

        private:
            void fail(const std::string &key, const std::string &reason) {
                if (error.empty()) {
                    error = "配置项 `" + key + "`: " + reason;
                }
            }

            const KeyValues &values;
            std::string &error;
        };

    }// namespace

    bool load_config_file(const std::string &path,
                          Parameters &parameters,
                          TransportConfig &transport_config,
                          std::string &error) {
        error.clear();
        KeyValues values;
        if (!flatten(path, values, error)) {
            return false;
        }
        Reader reader(values, error);

        // ---- 通信层 ----
        transport_config.kind = reader.string_of("transport", transport_config.kind);
        transport_config.lidar_topic = reader.string_of("lidar_topic", transport_config.lidar_topic);
        transport_config.imu_topic = reader.string_of("imu_topic", transport_config.imu_topic);
        transport_config.lidar_type = reader.string_of("lidar_type", transport_config.lidar_type);
        transport_config.lidar_frame = reader.string_of("lidar_frame", transport_config.lidar_frame);
        transport_config.save_pcd = reader.boolean("save_pcd", false, false);
        transport_config.record_path = reader.string_of("record_path", transport_config.record_path);
        transport_config.replay_path = reader.string_of("replay_path", transport_config.replay_path);
        transport_config.odometry_output_path =
                reader.string_of("odometry_output_path", transport_config.odometry_output_path);
        transport_config.pcd_output_path =
                reader.string_of("pcd_output_path", transport_config.pcd_output_path);

        // ---- 点云过滤 ----
        parameters.point_filter_num = static_cast<int>(reader.number("point_filter_num"));
        double min_distance = reader.number("min_distance");
        double max_distance = reader.number("max_distance");
        parameters.space_downsample = reader.boolean("space_downsample", true, true);
        parameters.space_downsample_leaf_size = static_cast<float>(reader.number("space_downsample_leaf_size"));

        // ---- IMU 处理 ----
        std::vector<double> gravity = reader.numbers("gravity", 3);
        parameters.gravity << gravity[0], gravity[1], gravity[2];
        parameters.check_satu = reader.boolean("check_satu", true, true);
        parameters.fix_gravity_direction = reader.boolean("fix_gravity_direction", true, true);
        double raw_satu_acc = reader.number("satu_acc");
        double raw_satu_gyro = reader.number("satu_gyro");
        parameters.acc_norm = reader.number("acc_norm");

        // ---- 地图 ----
        parameters.map_resolution = reader.number("map_resolution");
        parameters.init_map_size = static_cast<size_t>(reader.number("init_map_size"));

        // ---- 雷达与 IMU 相对位姿 ----
        parameters.extrinsic_est_en = reader.boolean("extrinsic_est_en", false, true);
        std::vector<double> extrinsic_T = reader.numbers("extrinsic_T", 3);
        parameters.extrinsic_T << extrinsic_T[0], extrinsic_T[1], extrinsic_T[2];
        std::vector<double> extrinsic_R = reader.numbers("extrinsic_R", 9);
        parameters.extrinsic_R << extrinsic_R[0], extrinsic_R[1], extrinsic_R[2],
                extrinsic_R[3], extrinsic_R[4], extrinsic_R[5],
                extrinsic_R[6], extrinsic_R[7], extrinsic_R[8];

        // ---- 滤波器 ----
        parameters.laser_point_cov = reader.number("laser_point_cov");
        parameters.imu_meas_acc_cov = reader.number("imu_meas_acc_cov");
        parameters.imu_meas_omg_cov = reader.number("imu_meas_omg_cov");
        parameters.velocity_cov = reader.number("velocity_cov");
        parameters.acceleration_cov = reader.number("acceleration_cov");
        parameters.omg_cov = reader.number("omg_cov");
        parameters.ba_cov = reader.number("ba_cov");
        parameters.bg_cov = reader.number("bg_cov");
        parameters.plane_threshold = reader.number("plane_threshold");
        parameters.match_sqaured = reader.number("match_sqaured");

        // ---- 数据发布 ----
        parameters.publish_odometry_without_downsample =
                reader.boolean("publish_odometry_without_downsample", false, true);

        if (!reader.ok()) {
            return false;
        }
        parameters.finalize(min_distance, max_distance, raw_satu_acc, raw_satu_gyro);
        return true;
    }

}// namespace small_point_lio::transport
