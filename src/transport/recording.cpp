#include "transport/recording.h"

#include "common/log.h"
#include "transport/mcap/cdr.h"
#include "transport/mcap/ros2_schemas.hpp"

#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

namespace small_point_lio::transport {

    namespace {

        // sensor_msgs/msg/PointField 的 datatype 常量
        constexpr uint8_t PF_FLOAT32 = 7;
        constexpr uint8_t PF_FLOAT64 = 8;

        constexpr uint32_t POINT_STRIDE = 20;// x,y,z float32 + t float64
        constexpr uint64_t NS_PER_S = 1000000000ULL;

        uint64_t to_nanoseconds(double seconds) {
            return static_cast<uint64_t>(seconds * 1e9);
        }

        /// 写失败只报第一次，不然一旦磁盘满会刷屏。
        void report_write_failure(const mcap::Status &status, uint64_t &failures, const char *what) {
            if (status.ok()) {
                return;
            }
            if (failures == 0) {
                SPL_LOG_ERROR(std::string("录制写入失败(") + what + "): " + status.message +
                              "（后续同类错误不再重复报告）");
            }
            ++failures;
        }

        struct RosTime {
            int32_t sec;
            uint32_t nanosec;
        };

        RosTime split_time(double seconds) {
            auto sec = static_cast<int32_t>(std::floor(seconds));
            auto nanosec = static_cast<uint32_t>((seconds - sec) * 1e9);
            return {sec, nanosec};
        }

        void write_header(cdr::Writer &writer, double timestamp, const std::string &frame_id) {
            RosTime time = split_time(timestamp);
            writer.i32(time.sec);
            writer.u32(time.nanosec);
            writer.string(frame_id);
        }

        double read_header(cdr::Reader &reader, std::string &frame_id) {
            int32_t sec = reader.i32();
            uint32_t nanosec = reader.u32();
            frame_id = reader.string();
            return static_cast<double>(sec) + static_cast<double>(nanosec) * 1e-9;
        }

        struct FieldLayout {
            uint32_t offset;
            uint8_t datatype;
            bool found = false;
        };

        /// PointField datatype 对应的字节宽度。0 表示不认识这个类型。
        uint32_t datatype_size(uint8_t datatype) {
            switch (datatype) {
                case 1:// INT8
                case 2:// UINT8
                    return 1;
                case 3:// INT16
                case 4:// UINT16
                    return 2;
                case 5:// INT32
                case 6:// UINT32
                case PF_FLOAT32:
                    return 4;
                case PF_FLOAT64:
                    return 8;
                default:
                    return 0;
            }
        }

        /// 字段必须完整落在一个点的 point_step 之内。
        /// 回放要吃外部录的包，offset/datatype 都是不可信输入，
        /// 不校验就直接 record + offset 会读到 payload 外面去。
        bool field_fits(const FieldLayout &field, uint32_t point_step) {
            if (!field.found) {
                return false;
            }
            uint32_t size = datatype_size(field.datatype);
            return size != 0 && field.offset <= point_step - size && point_step >= size;
        }

    }// namespace

    PointTimeMode point_time_mode_from_lidar_type(const std::string &lidar_type) {
        if (lidar_type == "livox_pointcloud2" || lidar_type == "livox_custom_msg") {
            return PointTimeMode::LivoxNanoseconds;
        }
        if (lidar_type == "custom_mid360_driver") {
            return PointTimeMode::AbsoluteSeconds;
        }
        if (lidar_type == "unilidar") {
            return PointTimeMode::RelativeToHeader;
        }
        return PointTimeMode::HeaderOnly;
    }

    // ------------------------------------------------------------------ 写

    struct RecordingWriter::Impl {
        mcap::McapWriter writer;
        mcap::ChannelId imu_channel = 0;
        mcap::ChannelId lidar_channel = 0;
        std::string frame_id;
        uint64_t write_failures = 0;
        uint32_t imu_sequence = 0;
        uint32_t lidar_sequence = 0;
        bool open = false;
    };

    RecordingWriter::RecordingWriter() : impl(std::make_unique<Impl>()) {}

    RecordingWriter::~RecordingWriter() {
        close();
    }

    bool RecordingWriter::is_open() const {
        return impl->open;
    }

    bool RecordingWriter::open(const std::string &path,
                               const std::string &imu_topic,
                               const std::string &lidar_topic,
                               const std::string &frame_id,
                               std::string &error) {
        close();
        mcap::McapWriterOptions options("ros2");
        options.compression = mcap::Compression::Zstd;
        mcap::Status status = impl->writer.open(path, options);
        if (!status.ok()) {
            error = "无法写入 MCAP: " + path + " (" + status.message + ")";
            return false;
        }

        mcap::Schema imu_schema("sensor_msgs/msg/Imu", "ros2msg", mcap_schemas::IMU_ROS2MSG);
        impl->writer.addSchema(imu_schema);
        mcap::Schema lidar_schema("sensor_msgs/msg/PointCloud2", "ros2msg", mcap_schemas::POINTCLOUD2_ROS2MSG);
        impl->writer.addSchema(lidar_schema);

        mcap::Channel imu_channel(imu_topic, "cdr", imu_schema.id);
        impl->writer.addChannel(imu_channel);
        impl->imu_channel = imu_channel.id;
        mcap::Channel lidar_channel(lidar_topic, "cdr", lidar_schema.id);
        impl->writer.addChannel(lidar_channel);
        impl->lidar_channel = lidar_channel.id;

        impl->frame_id = frame_id;
        impl->open = true;
        return true;
    }

    void RecordingWriter::write_imu(const common::ImuMsg &imu_msg) {
        if (!impl->open) {
            return;
        }
        cdr::Writer body;
        write_header(body, imu_msg.timestamp, impl->frame_id);
        // orientation：本项目不产生姿态观测，按 ROS 约定填单位四元数 + 协方差首元素 -1
        body.f64(0.0), body.f64(0.0), body.f64(0.0), body.f64(1.0);
        body.f64(-1.0);
        for (int i = 1; i < 9; ++i) {
            body.f64(0.0);
        }
        body.f64(imu_msg.angular_velocity.x());
        body.f64(imu_msg.angular_velocity.y());
        body.f64(imu_msg.angular_velocity.z());
        for (int i = 0; i < 9; ++i) {
            body.f64(0.0);
        }
        body.f64(imu_msg.linear_acceleration.x());
        body.f64(imu_msg.linear_acceleration.y());
        body.f64(imu_msg.linear_acceleration.z());
        for (int i = 0; i < 9; ++i) {
            body.f64(0.0);
        }

        mcap::Message message;
        message.channelId = impl->imu_channel;
        message.sequence = ++impl->imu_sequence;
        message.logTime = to_nanoseconds(imu_msg.timestamp);
        message.publishTime = message.logTime;
        message.data = reinterpret_cast<const std::byte *>(body.data().data());
        message.dataSize = body.data().size();
        report_write_failure(impl->writer.write(message), impl->write_failures, "IMU");
    }

    void RecordingWriter::write_pointcloud(const std::vector<common::Point> &pointcloud) {
        if (!impl->open || pointcloud.empty()) {
            return;
        }
        // 整帧的 header 时间取第一个点的时间，per-point 时间照原样存进 t 字段。
        double frame_timestamp = pointcloud.front().timestamp;

        cdr::Writer body;
        write_header(body, frame_timestamp, impl->frame_id);
        body.u32(1);                                           // height
        body.u32(static_cast<uint32_t>(pointcloud.size()));     // width
        body.u32(4);                                            // fields.size()
        const struct {
            const char *name;
            uint32_t offset;
            uint8_t datatype;
        } fields[4] = {{"x", 0, PF_FLOAT32}, {"y", 4, PF_FLOAT32}, {"z", 8, PF_FLOAT32}, {"t", 12, PF_FLOAT64}};
        for (const auto &field: fields) {
            body.string(field.name);
            body.u32(field.offset);
            body.u8(field.datatype);
            body.u32(1);// count
        }
        body.boolean(false);// is_bigendian
        body.u32(POINT_STRIDE);
        body.u32(POINT_STRIDE * static_cast<uint32_t>(pointcloud.size()));// row_step
        uint8_t *data = body.bytes_uninitialized(POINT_STRIDE * pointcloud.size());
        for (const auto &point: pointcloud) {
            float position[3] = {point.position.x(), point.position.y(), point.position.z()};
            std::memcpy(data, position, 12);
            std::memcpy(data + 12, &point.timestamp, 8);
            data += POINT_STRIDE;
        }
        body.boolean(true);// is_dense

        mcap::Message message;
        message.channelId = impl->lidar_channel;
        message.sequence = ++impl->lidar_sequence;
        message.logTime = to_nanoseconds(frame_timestamp);
        message.publishTime = message.logTime;
        message.data = reinterpret_cast<const std::byte *>(body.data().data());
        message.dataSize = body.data().size();
        report_write_failure(impl->writer.write(message), impl->write_failures, "点云");
    }

    void RecordingWriter::close() {
        if (impl->open) {
            impl->writer.close();
            impl->open = false;
            if (impl->write_failures != 0) {
                SPL_LOG_ERROR("录制期间共有 " + std::to_string(impl->write_failures) +
                              " 条消息写入失败，这份录制不完整");
            }
        }
    }

    // ------------------------------------------------------------------ 读

    struct RecordingReader::Impl {
        mcap::McapReader reader;
        std::unique_ptr<mcap::LinearMessageView> view;
        std::unique_ptr<mcap::LinearMessageView::Iterator> iterator;
        std::string imu_topic;
        std::string lidar_topic;
        PointTimeMode fallback_mode = PointTimeMode::HeaderOnly;
        bool open = false;
    };

    RecordingReader::RecordingReader() : impl(std::make_unique<Impl>()) {}

    RecordingReader::~RecordingReader() {
        close();
    }

    bool RecordingReader::open(const std::string &path,
                               const std::string &imu_topic,
                               const std::string &lidar_topic,
                               PointTimeMode fallback_mode,
                               std::string &error) {
        close();
        mcap::Status status = impl->reader.open(path);
        if (!status.ok()) {
            error = "打不开 MCAP: " + path + " (" + status.message + ")";
            return false;
        }
        impl->imu_topic = imu_topic;
        impl->lidar_topic = lidar_topic;
        impl->fallback_mode = fallback_mode;
        impl->view = std::make_unique<mcap::LinearMessageView>(impl->reader.readMessages());
        impl->iterator = std::make_unique<mcap::LinearMessageView::Iterator>(impl->view->begin());
        impl->open = true;
        return true;
    }

    void RecordingReader::close() {
        if (impl->open) {
            impl->iterator.reset();
            impl->view.reset();
            impl->reader.close();
            impl->open = false;
        }
    }

    RecordingReader::Kind RecordingReader::read_next(common::ImuMsg &imu_msg,
                                                     std::vector<common::Point> &pointcloud,
                                                     std::string &error) {
        if (!impl->open) {
            error = "录制文件未打开";
            return Kind::End;
        }
        try {
            while (*impl->iterator != impl->view->end()) {
                const mcap::MessageView &view = **impl->iterator;
                const std::string &topic = view.channel->topic;
                bool is_imu = (topic == impl->imu_topic);
                bool is_lidar = (topic == impl->lidar_topic);
                if (!is_imu && !is_lidar) {
                    ++*impl->iterator;
                    continue;
                }
                cdr::Reader body(reinterpret_cast<const uint8_t *>(view.message.data), view.message.dataSize);
                std::string frame_id;
                double header_timestamp = read_header(body, frame_id);

                if (is_imu) {
                    body.f64(), body.f64(), body.f64(), body.f64();// orientation
                    for (int i = 0; i < 9; ++i) {
                        body.f64();// orientation_covariance
                    }
                    double wx = body.f64(), wy = body.f64(), wz = body.f64();
                    for (int i = 0; i < 9; ++i) {
                        body.f64();// angular_velocity_covariance
                    }
                    double ax = body.f64(), ay = body.f64(), az = body.f64();
                    imu_msg.timestamp = header_timestamp;
                    imu_msg.angular_velocity = Eigen::Vector3d(wx, wy, wz);
                    imu_msg.linear_acceleration = Eigen::Vector3d(ax, ay, az);
                    ++*impl->iterator;
                    return Kind::Imu;
                }

                uint32_t height = body.u32();
                uint32_t width = body.u32();
                uint32_t field_count = body.u32();
                FieldLayout x_field, y_field, z_field, time_field;
                PointTimeMode mode = impl->fallback_mode;
                for (uint32_t i = 0; i < field_count; ++i) {
                    std::string name = body.string();
                    FieldLayout layout;
                    layout.offset = body.u32();
                    layout.datatype = body.u8();
                    body.u32();// count
                    layout.found = true;
                    if (name == "x") {
                        x_field = layout;
                    } else if (name == "y") {
                        y_field = layout;
                    } else if (name == "z") {
                        z_field = layout;
                    } else if (name == "t" && layout.datatype == PF_FLOAT64) {
                        // 本项目录制时写的规范字段，语义明确，优先于配置里的猜测
                        time_field = layout;
                        mode = PointTimeMode::CanonicalAbsoluteSeconds;
                    } else if (name == "timestamp" && !time_field.found) {
                        time_field = layout;
                    }
                }
                body.boolean();// is_bigendian
                uint32_t point_step = body.u32();
                body.u32();// row_step
                size_t data_size = 0;
                const uint8_t *data = body.bytes(data_size);

                if (!x_field.found || !y_field.found || !z_field.found) {
                    error = "点云里找不到 x/y/z 字段";
                    return Kind::End;
                }
                if (point_step == 0) {
                    error = "点云 point_step 为 0";
                    return Kind::End;
                }
                if (!field_fits(x_field, point_step) || !field_fits(y_field, point_step) ||
                    !field_fits(z_field, point_step)) {
                    error = "点云 x/y/z 字段的 offset/datatype 超出了 point_step";
                    return Kind::End;
                }
                if (x_field.datatype != PF_FLOAT32 || y_field.datatype != PF_FLOAT32 ||
                    z_field.datatype != PF_FLOAT32) {
                    error = "点云 x/y/z 字段不是 FLOAT32";
                    return Kind::End;
                }
                if (time_field.found && !field_fits(time_field, point_step)) {
                    error = "点云时间字段的 offset/datatype 超出了 point_step";
                    return Kind::End;
                }
                // height * width 与 count * point_step 都可能在 32 位上回绕，
                // 一律用除法比较，别用乘法。
                if (height != 0 && width > SIZE_MAX / height) {
                    error = "点云 height*width 溢出";
                    return Kind::End;
                }
                size_t count = static_cast<size_t>(height) * width;
                if (count > data_size / point_step) {
                    error = "点云 data 长度和 point_step/width 对不上";
                    return Kind::End;
                }
                pointcloud.clear();
                pointcloud.resize(count);
                for (size_t i = 0; i < count; ++i) {
                    const uint8_t *record = data + i * point_step;
                    float x, y, z;
                    std::memcpy(&x, record + x_field.offset, 4);
                    std::memcpy(&y, record + y_field.offset, 4);
                    std::memcpy(&z, record + z_field.offset, 4);
                    pointcloud[i].position << x, y, z;
                    double timestamp = header_timestamp;
                    if (time_field.found && mode != PointTimeMode::HeaderOnly) {
                        double raw = 0.0;
                        if (time_field.datatype == PF_FLOAT64) {
                            std::memcpy(&raw, record + time_field.offset, 8);
                        } else if (time_field.datatype == PF_FLOAT32) {
                            float value;
                            std::memcpy(&value, record + time_field.offset, 4);
                            raw = value;
                        }
                        switch (mode) {
                            case PointTimeMode::CanonicalAbsoluteSeconds:
                            case PointTimeMode::AbsoluteSeconds:
                                timestamp = raw;
                                break;
                            case PointTimeMode::LivoxNanoseconds:
                                timestamp = raw * 1e-9;
                                break;
                            case PointTimeMode::RelativeToHeader:
                                timestamp = header_timestamp + raw;
                                break;
                            case PointTimeMode::HeaderOnly:
                                break;
                        }
                    }
                    pointcloud[i].timestamp = timestamp;
                }
                ++*impl->iterator;
                return Kind::Pointcloud;
            }
        } catch (const std::exception &exception) {
            error = std::string("解析录制文件出错: ") + exception.what();
            return Kind::End;
        }
        return Kind::End;
    }

}// namespace small_point_lio::transport
